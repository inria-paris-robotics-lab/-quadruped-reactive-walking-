from tracemalloc import start
from .ProblemData import ProblemData
from .Target import Target
from .OcpResult import OcpResult
import crocoddyl
import pinocchio as pin
import numpy as np
from time import time

class OCP:
    def __init__(self, pd:ProblemData, target:Target):
        self.pd = pd
        self.target = target
        
        self.results = OcpResult()
        self.state = crocoddyl.StateMultibody(self.pd.model)
        self.initialized = False
        self.t_problem_update = 0
        self.initialize_models()
    
    def initialize_models(self):
        self.runningModels = []
        self.terminalModel = []
        for t in range(self.pd.T):
            self.runningModels.append(Model(self.pd, self.state, self.target.contactSequence[t])) # RunningModels
            target = self.target.evaluate_in_t(t)
            freeIds = [idf for idf in self.pd.allContactIds if idf not in self.target.contactSequence[t]]
            self.appendTargetToModel(self.runningModels[t], target, self.target.contactSequence[t], freeIds)

        self.bufferModel = self.runningModels[0]
        self.bufferData = self.runningModels[0].model.createData()

        self.terminalModel.append(Model(self.pd, self.state, isTerminal=True)) # TerminalModel

        self.models = self.runningModels + self.terminalModel
        
    def make_ocp(self, x0):
        """ Create a shooting problem for a simple walking gait.

        :param x0: initial state
        """
        
        # Compute the current foot positions
        q0 = x0[:self.pd.nq]
        pin.forwardKinematics(self.pd.model, self.pd.rdata, q0)
        pin.updateFramePlacements(self.pd.model, self.pd.rdata)

        start_time = time()
        if self.initialized:
            target = self.target.evaluate_in_t(self.pd.T-1)
            freeIds = [idf for idf in self.pd.allContactIds if idf not in self.target.contactSequence[self.pd.T-1]]        
            self.appendTargetToModel(self.bufferModel, target, self.target.contactSequence[self.pd.T-1], freeIds)
            self.problem.circularAppend(self.bufferModel.model, self.bufferModel.model.createData())
        else:
            self.problem = crocoddyl.ShootingProblem(x0, 
                                            [m.model for m in self.models[:-1]], 
                                            self.models[-1].model)
        
        
        self.t_problem_update = time() - start_time
        
        
        self.ddp = crocoddyl.SolverFDDP(self.problem)

        self.initialized = True

        return self.problem

    def appendTargetToModel(self, model, target, contactIds, freeIds):
        """ Action models for a footstep phase.
        :param numKnots: number of knots for the footstep phase
        :param supportFootIds: Ids of the supporting feet
        :param swingFootIds: Ids of the swinging foot
        :return footstep action models
        """
        # Action models for the foot swing
        swingFootTask = []
        for i in freeIds:
            try:
                tref = target[i]
                swingFootTask += [[i, pin.SE3(np.eye(3), tref)]]
            except:
                pass

        
        model.update_model(contactIds, swingFootTask)


# Solve
    def solve(self, x0, guess=None):
        problem = self.make_ocp(x0)
        self.ddp = crocoddyl.SolverFDDP(problem)
        # self.ddp.setCallbacks([crocoddyl.CallbackVerbose()])

        # for i, c in enumerate(self.ddp.problem.runningModels):
        #     print(str(i), c.differential.contacts.contacts.todict().keys())
        # print(str(i+1), self.ddp.problem.terminalModel.differential.contacts.contacts.todict().keys())
        # print("\n")

        if not guess:
            print("No warmstart provided")
            xs = [x0] * (self.ddp.problem.T + 1)
            us = self.ddp.problem.quasiStatic([x0] * self.ddp.problem.T)
        else:
            xs = guess['xs']
            us = guess['us']
            print("Using warmstart")
        start_time = time()
        self.ddp.solve(xs, us, 1, False)
        self.solver_time = time()- start_time
        print("Solver time: ", self.solver_time)

    def get_results(self):
        self.results.x = self.ddp.xs.tolist()
        self.results.a = self.get_croco_acc()
        self.results.u = self.ddp.us.tolist()
        self.results.K = self.ddp.K
        self.results.solver_time = self.solver_time
        return self.results

    def get_croco_forces(self):
        d = self.ddp.problem.runningDatas[0]
        cnames = d.differential.multibody.contacts.contacts.todict().keys()
        forces = {n: [] for n in cnames}

        for m in self.ddp.problem.runningDatas:
            mdict = m.differential.multibody.contacts.contacts.todict()
            for n in cnames:
                if n in mdict:
                    forces[n] += [(mdict[n].jMf.inverse()*mdict[n].f).linear]
                else:
                    forces[n] += [np.array([0, 0, 0])]
        for f in forces:
            forces[f] = np.array(forces[f])
        return forces

    def get_croco_forces_ws(self):
        forces = []

        for m in self.ddp.problem.runningDatas:
            mdict = m.differential.multibody.contacts.contacts.todict()
            f_tmp = []
            for n in mdict:
                f_tmp += [(mdict[n].jMf.inverse()*mdict[n].f).linear]
            forces += [np.concatenate(f_tmp)]
        return forces

    def get_croco_acc(self):
        acc = []
        [acc.append(m.differential.xout)
         for m in self.ddp.problem.runningDatas]
        return acc

    

class Model:
    def __init__(self, pd, state, supportFootIds=[], isTerminal=False):
        self.pd = pd
        self.supportFootIds=supportFootIds
        self.isTerminal=isTerminal

        self.state = state
        if pd.useFixedBase == 0:
            self.actuation = crocoddyl.ActuationModelFloatingBase(self.state)
        else:
            self.actuation = crocoddyl.ActuationModelFull(self.state)
        self.control = crocoddyl.ControlParametrizationModelPolyZero(self.actuation.nu)
        self.nu = self.actuation.nu

        self.createStandardModel()
        if isTerminal:
            self.make_terminal_model()
        else:
            self.make_running_model()

    def createStandardModel(self):
        """ Action model for a swing foot phase.

        :param timeStep: step duration of the action model
        :param supportFootIds: Ids of the constrained feet
        :param comTask: CoM task
        :param swingFootTask: swinging foot task
        :return action model for a swing foot phase
        """
        
        self.contactModel = crocoddyl.ContactModelMultiple(self.state, self.nu)
        for i in self.supportFootIds:
            supportContactModel = crocoddyl.ContactModel3D(self.state, i, np.array([0., 0., 0.]), self.nu,
                                                           np.array([0., 0.]))
            self.contactModel.addContact(self.pd.model.frames[i].name + "_contact", supportContactModel)

        # Creating the cost model for a contact phase
        costModel = crocoddyl.CostModelSum(self.state, self.nu)

        stateResidual = crocoddyl.ResidualModelState(self.state, self.pd.xref, self.nu)
        stateActivation = crocoddyl.ActivationModelWeightedQuad(self.pd.state_reg_w**2)
        stateReg = crocoddyl.CostModelResidual(self.state, stateActivation, stateResidual)
        costModel.addCost("stateReg", stateReg, 1)

        self.costModel = costModel

        self.dmodel = crocoddyl.DifferentialActionModelContactFwdDynamics(self.state, self.actuation, self.contactModel,
                                                                    self.costModel, 0., True)
        self.model = crocoddyl.IntegratedActionModelEuler(self.dmodel, self.control, self.pd.dt)

    def update_contact_model(self):
        self.remove_contacts()
        self.contactModel = crocoddyl.ContactModelMultiple(self.state, self.nu)
        for i in self.supportFootIds:
            supportContactModel = crocoddyl.ContactModel3D(self.state, i, np.array([0., 0., 0.]), self.nu,
                                                           np.array([0., 0.]))
            self.dmodel.contacts.addContact(self.pd.model.frames[i].name + "_contact", supportContactModel)
    
    def make_terminal_model(self):
        self.remove_running_costs()  
        self.update_contact_model()

        self.isTerminal=True
        stateResidual = crocoddyl.ResidualModelState(self.state, self.pd.xref, self.nu)
        stateActivation = crocoddyl.ActivationModelWeightedQuad(self.pd.terminal_velocity_w**2)
        stateReg = crocoddyl.CostModelResidual(self.state, stateActivation, stateResidual)
        self.costModel.addCost("terminalVelocity", stateReg, 1)

    def make_running_model(self):
        self.remove_terminal_cost()

        self.isTerminal = False

        self.update_contact_model()
        for i in self.supportFootIds:
            cone = crocoddyl.FrictionCone(self.pd.Rsurf, self.pd.mu, 4, False)
            coneResidual = crocoddyl.ResidualModelContactFrictionCone(self.state, i, cone, self.nu)
            coneActivation = crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(cone.lb, cone.ub))
            frictionCone = crocoddyl.CostModelResidual(self.state, coneActivation, coneResidual)
            self.costModel.addCost(self.pd.model.frames[i].name + "_frictionCone", frictionCone, self.pd.friction_cone_w)

        ctrlResidual = crocoddyl.ResidualModelControl(self.state, self.pd.uref)
        ctrlReg = crocoddyl.CostModelResidual(self.state, ctrlResidual)
        self.costModel.addCost("ctrlReg", ctrlReg, self.pd.control_reg_w)

        ctrl_bound_residual = crocoddyl.ResidualModelControl(self.state, self.nu)
        ctrl_bound_activation = crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(-self.pd.effort_limit, self.pd.effort_limit))
        ctrl_bound = crocoddyl.CostModelResidual(self.state, ctrl_bound_activation, ctrl_bound_residual)
        self.costModel.addCost("ctrlBound", ctrl_bound, self.pd.control_bound_w)
    
    def remove_running_costs(self):
        runningCosts = self.dmodel.costs.active.tolist()
        idx = runningCosts.index("stateReg")
        runningCosts.pop(idx)
        for cost in runningCosts:
            if cost in self.dmodel.costs.active.tolist():
                self.dmodel.costs.removeCost(cost)

    def remove_terminal_cost(self):
        if "terminalVelocity" in self.dmodel.costs.active.tolist():
                self.dmodel.costs.removeCost("terminalVelocity")

    def remove_contacts(self):
        allContacts = self.dmodel.contacts.contacts.todict()
        for c in allContacts:
            self.dmodel.contacts.removeContact(c)

    def tracking_cost(self, swingFootTask):
        if swingFootTask is not None:
            for i in swingFootTask:
                frameTranslationResidual = crocoddyl.ResidualModelFrameTranslation(self.state, i[0], i[1].translation,self.nu)
                footTrack = crocoddyl.CostModelResidual(self.state, frameTranslationResidual)
                if self.pd.model.frames[i[0]].name + "_footTrack" in self.dmodel.costs.active.tolist():
                    self.dmodel.costs.removeCost(self.pd.model.frames[i[0]].name + "_footTrack")
                self.costModel.addCost(self.pd.model.frames[i[0]].name + "_footTrack", footTrack, self.pd.foot_tracking_w)

    def update_model(self, supportFootIds = [], swingFootTask=[]):
        self.supportFootIds = supportFootIds
        self.tracking_cost(swingFootTask)
