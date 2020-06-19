#!/usr/bin/env python
import random
from netpyne import specs, sim
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt
import scipy.io as sio
from scipy.signal import argrelmax
import pylab
from std_msgs.msg import Float64
from rat_model.msg import list_pop
import rospy
from std_msgs.msg import Float64MultiArray,String

class Network:
    class Spikes:
        def __init__(self):
            self.times = []

    def __init__(self,
                 has_pd=0,
                 it_num=1,
                 dbs=False,
                 t_sim=1000,
                 n_neurons=10):

        # Variables
        self.pd = has_pd
        self.dbs = dbs
        self.t_sim = t_sim
        self.amplitude=1.2
        self.init=False

        #Subscriber
        rospy.Subscriber('/stimulation',Float64, self.neuralModel)
        
        # Parameters
        self.netParams = self.buildNetParams()
        self.buildPopulationParameters()
        self.buildCellRules()
        self.buildSynMechParams()
        self.buildCellConnRules()
        self.buildStimParams()
        self.avpopratesregions=[]

        #ROS
        # Rate (Hz)
        self.loop_rate = rospy.Rate(1)
        #Publisher
        self.pub=rospy.Publisher('/avpoprates',list_pop, queue_size=100)


    def neuralModel(self,data):
        # Modify stimulus

        self.amplitude= data.data
        self.init= True
        """
        if data.data == 1.8:
            sim.net.modifyStims({'conds':{'source':'Input_th'},'cellConds':{'pop':'TH'},'amp':0.0018})
            #self.stimulus_moment['stim'].append('stimulated')
            #self.stimulus_moment['moment'].append(self.current_interval)
        else:
            sim.net.modifyStims({'conds':{'source':'Input_th'},'cellConds':{'pop':'TH'},'amp':0.0012})
            #self.stimulus_moment['stim'].append('Notstimulated')
            #self.stimulus_moment['moment'].append(self.current_interval)  
        """   

    def buildNetParams(self):
        return specs.NetParams()  # object of class NetParams to store the network parameters

    def buildPopulationParameters(self,
                                  n_strd1=10, n_strd2=10,
                                  n_th=10, n_gpi=10,
                                  n_gpe=10, n_rs=10,
                                  n_fsi=10, n_stn=10):

        self.netParams.sizeX = 7500  # x-dimension (horizontal length) size in um
        self.netParams.sizeY = 8800  # y-dimension (vertical height or cortical depth) size in um
        self.netParams.sizeZ = 5000  # z-dimension (horizontal length) size in um

        self.netParams.popParams['StrD1'] = {'cellModel': 'StrD1',
                                             'cellType': 'StrD1',
                                             'numCells': n_strd1,
                                             'xRange': [4000, 6000],
                                             'yRange': [3900, 5900],
                                             'zRange': [3000, 5000]}
        self.netParams.popParams['StrD2'] = {'cellModel': 'StrD2',
                                             'cellType': 'StrD2',
                                             'numCells': n_strd2,
                                             'xRange': [4000, 6000],
                                             'yRange': [3900, 5900],
                                             'zRange': [3000, 5000]}
        # considering VPL coordinates
        self.netParams.popParams['TH'] = {'cellModel': 'TH',
                                          'cellType': 'Thal',
                                          'numCells': n_th,
                                          'xRange': [0, 2000],
                                          'yRange': [1600, 3600],
                                          'zRange': [800, 2800]}
        self.netParams.popParams['GPi'] = {'cellModel': 'GPi',
                                           'cellType': 'GPi',
                                           'numCells': n_gpi,
                                           'xRange': [3500, 5500],
                                           'yRange': [200, 2200],
                                           'zRange': [0, 2000]}
        self.netParams.popParams['GPe'] = {'cellModel': 'GPe',
                                           'cellType': 'GPe',
                                           'numCells': n_gpe,
                                           'xRange': [3500, 5500],
                                           'yRange': [1200, 3200],
                                           'zRange': [1700, 3700]}
        # considering M1
        self.netParams.popParams['CTX_RS'] = {'cellModel': 'CTX_RS',
                                              'cellType': 'CTX_RS',
                                              'numCells': n_rs,
                                              'xRange': [5500, 7500],
                                              'yRange': [6800, 8800],
                                              'zRange': [3000, 5000]}
        self.netParams.popParams['CTX_FSI'] = {'cellModel': 'CTX_FSI',
                                               'cellType': 'CTX_FSI',
                                               'numCells': n_fsi,
                                               'xRange': [5500, 7500],
                                               'yRange': [6800, 8800],
                                               'zRange': [3000, 5000]}
        self.netParams.popParams['STN'] = {'cellModel': 'STN',
                                           'cellType': 'STN',
                                           'numCells': n_stn,
                                           'xRange': [1000, 3000],
                                           'yRange': [0, 2000],
                                           'zRange': [200, 2200]}

    def buildCellRules(self, **args):
        self.rsCellRules(**args)
        self.fsiCellRules(**args)
        self.strD1CellRules(**args)
        self.strD2CellRules(**args)
        self.thCellRules(**args)
        self.gpiCellRules(**args)
        self.gpeCellRules(**args)
        self.stnCellRules(**args)

    def rsCellRules(self):
        cellRule = {'conds': {'cellModel': 'CTX_RS', 'cellType': 'CTX_RS'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'pointps': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1,
                                            'cm': 1}
        cellRule['secs']['soma']['pointps']['Izhi'] = {'mod': 'Izhi2003b',
                                                       'a': 0.02,
                                                       'b': 0.2,
                                                       'c': -65,
                                                       'd': 8,
                                                       'f': 5,
                                                       'g': 140,
                                                       'thresh': 30}
        cellRule['secs']['soma']['vinit'] = -65
        cellRule['secs']['soma']['threshold'] = 30
        self.netParams.cellParams['CTX_RS'] = cellRule

    def fsiCellRules(self):
        cellRule = {'conds': {'cellModel': 'CTX_FSI', 'cellType': 'CTX_FSI'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'pointps': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1,
                                            'cm': 1}
        cellRule['secs']['soma']['pointps']['Izhi'] = {'mod': 'Izhi2003b',
                                                       'a': 0.1,
                                                       'b': 0.2,
                                                       'c': -65,
                                                       'd': 2,
                                                       'f': 5,
                                                       'g': 140,
                                                       'thresh': 30}
        cellRule['secs']['soma']['vinit'] = -65
        cellRule['secs']['soma']['threshold'] = 30
        self.netParams.cellParams['CTX_FSI'] = cellRule

    def strD1CellRules(self):
        cellRule = {'conds': {'cellModel': 'StrD1', 'cellType': 'StrD1'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['Str'] = {'gmbar': (2.6e-3 - self.pd * 1.1e-3)}
        cellRule['secs']['soma']['vinit'] = random.gauss(-63.8, 5)
        cellRule['secs']['soma']['threshold'] = -10
        self.netParams.cellParams['StrD1'] = cellRule

    def strD2CellRules(self):
        cellRule = {'conds': {'cellModel': 'StrD2', 'cellType': 'StrD2'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['Str'] = {'gmbar': (2.6e-3 - self.pd * 1.1e-3)}
        cellRule['secs']['soma']['vinit'] = random.gauss(-63.8, 5)
        cellRule['secs']['soma']['threshold'] = -10
        self.netParams.cellParams['StrD2'] = cellRule

    def thCellRules(self):
        cellRule = {'conds': {'cellModel': 'TH', 'cellType': 'Thal'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['thalamus'] = {}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10
        self.netParams.cellParams['TH'] = cellRule

    def gpiCellRules(self, gahp=10e-3):
        cellRule = {'conds': {'cellModel': 'GPi', 'cellType': 'GPi'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['GP'] = {'gahp': gahp}
        # cellRule['secs']['GPi']['mechs']['GP'] = {}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10
        self.netParams.cellParams['GPi'] = cellRule

    def gpeCellRules(self, gahp=10e-3):
        cellRule = {'conds': {'cellModel': 'GPe', 'cellType': 'GPe'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['GP'] = {'gahp': gahp}
        # cellRule['secs']['GPe']['mechs']['GP'] = {}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10
        self.netParams.cellParams['GPe'] = cellRule

    def stnCellRules(self, gkcabar=5e-3):
        cellRule = {'conds': {'cellModel': 'STN', 'cellType': 'STN'}, 'secs': {}}
        cellRule['secs']['soma'] = {'geom': {}, 'mechs': {}}
        cellRule['secs']['soma']['geom'] = {'diam': 5.642,
                                            'L': 5.642,
                                            'Ra': 1,
                                            'nseg': 1}
        cellRule['secs']['soma']['mechs']['STN'] = {'dbs': self.dbs,
                                                    'gkcabar': gkcabar}
        cellRule['secs']['soma']['vinit'] = random.gauss(-62, 5)
        cellRule['secs']['soma']['threshold'] = -10
        self.netParams.cellParams['STN'] = cellRule

    def buildSynMechParams(self):
        # TH
        self.netParams.synMechParams['Igith'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': -85}  # gpi -<th
        # GPe
        self.netParams.synMechParams['Insge,ampa'] = {'mod': 'Exp2Syn',
                                                      'tau1': 0.4,
                                                      'tau2': 2.5,
                                                      'e': 0}  # stn -> gpe
        self.netParams.synMechParams['Insge,nmda'] = {'mod': 'Exp2Syn',
                                                      'tau1': 2,
                                                      'tau2': 67,
                                                      'e': 0}  # stn -> gpe
        self.netParams.synMechParams['Igege'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': -85}  # gpe -< gpe
        self.netParams.synMechParams['Istrgpe'] = {'mod': 'Exp2Syn',
                                                   'tau1': 5,
                                                   'tau2': 5,
                                                   'e': -85}  # D2 -> gpe
        # GPi
        self.netParams.synMechParams['Igegi'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': -85}  # gpe -< gp
        self.netParams.synMechParams['Isngi'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': 0}  # stn -> gpi
        self.netParams.synMechParams['Istrgpi'] = {'mod': 'Exp2Syn',
                                                   'tau1': 5,
                                                   'tau2': 5,
                                                   'e': -85}  # D1 -> gpi
        # STN
        self.netParams.synMechParams['Igesn'] = {'mod': 'Exp2Syn',
                                                 'tau1': 0.4,
                                                 'tau2': 7.7,
                                                 'e': -85}  # gpe -< stn
        self.netParams.synMechParams['Icosn,ampa'] = {'mod': 'Exp2Syn',
                                                      'tau1': 0.5,
                                                      'tau2': 2.49,
                                                      'e': 0}  # ctx -> gpe
        self.netParams.synMechParams['Icosn,nmda'] = {'mod': 'Exp2Syn',
                                                      'tau1': 2,
                                                      'tau2': 90,
                                                      'e': 0}  # ctx -> gpe
        # Str
        self.netParams.synMechParams['Igabadr'] = {'mod': 'Exp2Syn',
                                                   'tau1': 0.1,
                                                   'tau2': 13,
                                                   'e': -80}  # str -< str
        self.netParams.synMechParams['Igabaindr'] = {'mod': 'Exp2Syn',
                                                     'tau1': 0.1,
                                                     'tau2': 13,
                                                     'e': -80}  # str -< str
        self.netParams.synMechParams['Icostr'] = {'mod': 'Exp2Syn',
                                                  'tau1': 5,
                                                  'tau2': 5,
                                                  'e': 0}  # ctx -> str
        # CTX
        self.netParams.synMechParams['Iei'] = {'mod': 'Exp2Syn',
                                               'tau1': 5,
                                               'tau2': 5,
                                               'e': 0}  # rs->fsi
        self.netParams.synMechParams['Iie'] = {'mod': 'Exp2Syn',
                                               'tau1': 5,
                                               'tau2': 5,
                                               'e': -85}  # fsi<-rs
        self.netParams.synMechParams['Ithco'] = {'mod': 'Exp2Syn',
                                                 'tau1': 5,
                                                 'tau2': 5,
                                                 'e': 0}  # th->rs

    def buildCellConnRules(self, **args):
        self.thConnRules(**args)
        self.gpeConnRules(**args)
        self.gpiConnRules(**args)
        self.stnConnRules(**args)
        self.strConnRules(**args)
        self.ctxConnRules(**args)

    def thConnRules(self, **args):
        # GPi-> Th connections 
        n_neurons = min(self.netParams.popParams['TH']['numCells'],
                        self.netParams.popParams['GPi']['numCells'])
        self.netParams.connParams['GPi->th'] = {
            'preConds': {'pop': 'GPi'}, 'postConds': {'pop': 'TH'},  # GPi-> th
            'connList': [[i, i] for i in range(n_neurons)],
            'weight': 0.0336e-3,  # synaptic weight (conductance)
            'delay': 5,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igith'}  # target synaptic mechanism

    def gpeConnRules(self,
                     stn_gpe=2,
                     gpe_gpe=2,
                     **args):
        # STN->GPe connections
        # Two aleatory GPe cells (index i) receive synapse from cells i and i - 1
        n_neurons = min(self.netParams.popParams['STN']['numCells'],
                        self.netParams.popParams['GPe']['numCells'])
        aux = random.sample(range(n_neurons), stn_gpe)
        connList = [[x - c, x] for x in aux for c in [1, 0]]
        weight = [random.uniform(0, 0.3) * 0.43e-3 for k in range(len(connList))]
        self.netParams.connParams['STN->GPe'] = {
            'preConds': {'pop': 'STN'}, 'postConds': {'pop': 'GPe'},  # STN-> GPe
            'connList': connList,  # AMPA
            'weight': weight,  # synaptic weight (conductance)
            'delay': 2,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Insge,ampa'}  # target synaptic mechanism
        # STN->GPe connections
        # Two aleatory GPe cells (index i) receive synapse from cells i and i - 1
        aux = random.sample(range(n_neurons), stn_gpe)
        connList = [[x - c, x] for x in aux for c in [1, 0]]
        weight = [random.uniform(0, 0.002) * 0.43e-3 for k in range(len(connList))]
        self.netParams.connParams['STN->GPe2'] = {
            'preConds': {'pop': 'STN'}, 'postConds': {'pop': 'GPe'},  # STN-> GPe
            'connList': connList,  # NMDA
            'weight': weight,  # synaptic weight (conductance)
            'delay': 2,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Insge,nmda'}  # target synaptic mechanism

        # GPe-< GPe connections
        n_neurons = self.netParams.popParams['GPe']['numCells']
        connList = [[(idx + ncn) % n_neurons, idx] for ncn in range(1, gpe_gpe + 1, 2)
                    for idx in range(n_neurons)] + \
                   [[idx, (idx + ncn) % n_neurons] for ncn in range(2, gpe_gpe + 1, 2)
                    for idx in range(n_neurons)]
        # connList = [[2,1],[3,2],[4,3],[5,4],[6,5],[7,6],[8,7],[9,8],[0,9],[1,0],
        #            [8,0],[9,1],[0,2],[1,3],[2,4],[3,5],[4,6],[5,7],[6,8],[7,9]]
        weight = [(0.25 + 0.75 * self.pd) * random.uniform(0, 1) * 0.3e-3 \
                  for k in range(len(connList))]
        self.netParams.connParams['GPe->GPe'] = {
            'preConds': {'pop': 'GPe'}, 'postConds': {'pop': 'GPe'},  # GPe-< GPe
            'connList': connList,
            'weight': weight,  # synaptic weight (conductance)
            'delay': 1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igege'}  # target synaptic mechanism

        # StrD2>GPe connections
        n_strd2 = self.netParams.popParams['StrD2']['numCells']
        n_gpe = self.netParams.popParams['GPe']['numCells']
        self.netParams.connParams['StrD2->GPe'] = {
            'preConds': {'pop': 'StrD2'}, 'postConds': {'pop': 'GPe'},  # StrD2-> GPe
            'connList': [[j, i] for i in range(n_gpe)
                         for j in range(n_strd2)],
            'weight': 0.15e-3,  # synaptic weight (conductance)
            'delay': 5,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Istrgpe'}  # target synaptic mechanism

    def gpiConnRules(self,
                     stn_gpi=5,
                     gpe_gpi=2,
                     **args):
        # STN-> GPi connections
        # Five aleatory GPi cells (index i) receive synapse from cells i and i - 1
        n_neurons = min(self.netParams.popParams['STN']['numCells'],
                        self.netParams.popParams['GPi']['numCells'])
        aux = random.sample(range(n_neurons), stn_gpi)
        connList = [[x - c, x] for x in aux for c in [1, 0]]
        self.netParams.connParams['STN->GPi'] = {
            'preConds': {'pop': 'STN'}, 'postConds': {'pop': 'GPi'},
            'connList': connList,
            'weight': 0.0645e-3,  # synaptic weight (conductance)
            'delay': 1.5,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Isngi'}  # target synaptic mechanism

        # GPe-< GPi connections 
        n_neurons = min(self.netParams.popParams['GPe']['numCells'],
                        self.netParams.popParams['GPi']['numCells'])
        self.netParams.connParams['GPe->GPi'] = {
            'preConds': {'pop': 'GPe'}, 'postConds': {'pop': 'GPi'},
            'connList':
                [[idx, (idx + ncn) % n_neurons] for ncn in range(2, gpe_gpi + 1, 2)
                 for idx in range(n_neurons)] + \
                [[(idx + ncn) % n_neurons, idx] for ncn in range(1, gpe_gpi + 1, 2)
                 for idx in range(n_neurons)],
            # [ [ idx, (idx+2) % n_neurons ] for idx in range( n_neurons ) ] + \
            # [ [ (idx+1) % n_neurons, idx ] for idx in range( n_neurons ) ],
            'weight': 0.15e-3,  # synaptic weight (conductance)
            'delay': 3,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igegi'}  # target synaptic mechanism

        # StrD1>GPi connections
        n_strd1 = self.netParams.popParams['StrD1']['numCells']
        n_gpi = self.netParams.popParams['GPi']['numCells']
        self.netParams.connParams['StrD1->GPe'] = {
            'preConds': {'pop': 'StrD1'}, 'postConds': {'pop': 'GPi'},  # StrD1-> GPi
            'connList': [[j, i] for i in range(n_gpi)
                         for j in range(n_strd1)],
            'weight': 0.15e-3,  # synaptic weight (conductance)
            'delay': 4,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Istrgpi'}  # target synaptic mechanism

    def stnConnRules(self, **args):
        # GPe-> STN connections 
        n_neurons = min(self.netParams.popParams['GPe']['numCells'],
                        self.netParams.popParams['STN']['numCells'])
        self.netParams.connParams['GPe->STN'] = {
            'preConds': {'pop': 'GPe'}, 'postConds': {'pop': 'STN'},  # GPe-< STN
            'connList': [[(i + c) % n_neurons, i] for c in [1, 0] for i in range(n_neurons)],
            'weight': 0.15e-3,  # synaptic weight (conductance)
            'delay': 4,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igesn'}  # target synaptic mechanism

        # CTX-> STN connections
        n_neurons = min(self.netParams.popParams['CTX_RS']['numCells'],
                        self.netParams.popParams['STN']['numCells'])
        connList = [[(i + c) % n_neurons, i] for c in [1, 0] for i in range(n_neurons)]
        weight = [random.uniform(0, 0.3) * 0.43e-3 for k in range(len(connList))]
        self.netParams.connParams['CTX->STN'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'STN'},  # CTX-> STN
            'connList': connList,
            'weight': weight,  # synaptic weight (conductance)
            'delay': 5.9,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Icosn,ampa'}  # target synaptic mechanism
        # CTX-> STN2 
        connList = [[(i + c) % n_neurons, i] for c in [1, 0] for i in range(n_neurons)]
        weight = [random.uniform(0, 0.003) * 0.43e-3 for k in range(len(connList))]
        self.netParams.connParams['CTX->STN2'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'STN'},  # CTX-> STN
            'connList': connList,
            'weight': weight,  # synaptic weight (conductance)
            'delay': 5.9,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Icosn,nmda'}  # target synaptic mechanism

    def strConnRules(self,
                     strd2_strd2=4,
                     strd1_strd1=3,
                     gsynmod=1,
                     **args):
        # StrD2-< StrD2 connections
        # Each StrD2 cell receive synapse from 4 aleatory StrD2 cell (except from itself)
        n_neurons = self.netParams.popParams['StrD2']['numCells']
        connList = [[x, i] for i in range(n_neurons)
                    for x in random.sample([k for k in range(n_neurons) if k != i],
                                           strd2_strd2)]
        self.netParams.connParams['StrD2-> StrD2'] = {
            'preConds': {'pop': 'StrD2'}, 'postConds': {'pop': 'StrD2'},  # StrD2-< StrD2
            'connList': connList,
            'weight': 0.1 / 4 * 0.5e-3,  # synaptic weight (conductance) -> mudar essa maluquisse
            'delay': 0,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igabaindr'}  # target synaptic mechanism

        # StrD1-< StrD1 connections
        # Each StrD1 cell receive synapse from 3 aleatory StrD1 cell (except from itself)
        n_neurons = self.netParams.popParams['StrD1']['numCells']
        connList = [[x, i] for i in range(n_neurons)
                    for x in random.sample([k for k in range(n_neurons) if k != i],
                                           strd1_strd1)]
        self.netParams.connParams['StrD1-> StrD1'] = {
            'preConds': {'pop': 'StrD1'}, 'postConds': {'pop': 'StrD1'},  # StrD1-< StrD1
            'connList': connList,
            'weight': 0.1 / 3 * 0.5e-3,  # synaptic weight (conductance) -> mudar aqui tb
            'delay': 0,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Igabadr'}  # target synaptic mechanism

        # RS-> StrD1 connections 
        n_neurons = min(self.netParams.popParams['CTX_RS']['numCells'],
                        self.netParams.popParams['StrD1']['numCells'])
        self.netParams.connParams['RS->StrD1'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'StrD1'},  # RS-> StrD1
            'connList': [[i, i] for i in range(n_neurons)],
            'weight': (0.07 - 0.044 * self.pd) * 0.43e-3 * gsynmod,  # synaptic weight (conductance)
            'delay': 5.1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Icostr'}  # target synaptic mechanism

        # RS-> StrD2 connections 
        n_neurons = min(self.netParams.popParams['CTX_RS']['numCells'],
                        self.netParams.popParams['StrD2']['numCells'])
        self.netParams.connParams['RS->StrD2'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'StrD2'},  # RS-> StrD2 
            'connList': [[i, i] for i in range(n_neurons)],
            'weight': 0.07 * 0.43e-3 * gsynmod,  # synaptic weight (conductance)
            'delay': 5.1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Icostr'}  # target synaptic mechanism

    def ctxConnRules(self,
                     rs_fsi=4,
                     fsi_rs=4,
                     **args):
        # RS -> FSI connections
        # Each FSI cell receive synapse from 4 aleatory RS cells
        n_rs = self.netParams.popParams['CTX_RS']['numCells']
        n_fsi = self.netParams.popParams['CTX_FSI']['numCells']
        connList = [[x, i] for i in range(n_fsi)
                    for x in random.sample([k for k in range(n_rs) if k != i],
                                           rs_fsi)]
        self.netParams.connParams['ctx_rs->ctx_fsi'] = {
            'preConds': {'pop': 'CTX_RS'}, 'postConds': {'pop': 'CTX_FSI'},  # ctx_rs -> ctx_fsi
            'connList': connList,
            'weight': 0.043e-3,  # synaptic weight (conductance)
            'delay': 1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Iei'}  # target synaptic mechanism

        # FSI -> RS connections
        # Each RS cell receive synapse from 4 aleatory FSI cells
        connList = [[x, i] for i in range(n_rs)
                    for x in random.sample([k for k in range(n_fsi) if k != i],
                                           fsi_rs)]
        self.netParams.connParams['ctx_fsi->ctx_rs'] = {
            'preConds': {'pop': 'CTX_FSI'}, 'postConds': {'pop': 'CTX_RS'},  # ctx_fsi -< ctx_rs
            'connList': connList,
            'weight': 0.083e-3,  # synaptic weight (conductance)
            'delay': 1,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Iie'}  # target synaptic mechanism

        # Th -> RS connections
        n_neurons = min(self.netParams.popParams['TH']['numCells'],
                        self.netParams.popParams['CTX_RS']['numCells'])
        self.netParams.connParams['th->ctx_rs'] = {
            'preConds': {'pop': 'TH'}, 'postConds': {'pop': 'CTX_RS'},  # th -> ctx_rs
            'connList': [[i, i] for i in range(n_neurons)],
            'weight': 0.0645e-3,  # synaptic weight (conductance)
            'delay': 5,  # transmission delay (ms)
            'loc': 1,  # location of synapse
            'synMech': 'Ithco'}  # target synaptic mechanism

    def buildStimParams(self,
                        amp_th=1.2e-3, amp_gpe=3e-3,
                        amp_gpi=3e-3, amp_stn=0,
                        amp_fs=0, amp_rs=0,
                        amp_dstr=0, amp_istr=0):
        bin_fs = 0;
        bin_rs = 0;
        bin_gpe = 0;
        bin_gpi = 0;
        bin_stn = 0;
        bin_dstr = 0;
        bin_istr = 0;
        bin_th = 0;

        while self.init==False:
            continue

        # FS receve a constante 3 density current or 1 during cortical stimulation
        self.netParams.stimSourceParams['Input_FS'] = {'type': 'IClamp',
                                                       'delay': 0,
                                                       'dur': self.t_sim,
                                                       'amp': bin_fs * -1}
        self.netParams.stimTargetParams['Input_FS->FS'] = {'source': 'Input_FS',
                                                           'conds': {'pop': 'CTX_FSI'},
                                                           'sec': 'fsi',
                                                           'loc': 0}

        # RS receve a constante 3 density current or 1 during cortical stimulation
        self.netParams.stimSourceParams['Input_RS'] = {'type': 'IClamp',
                                                       'delay': 0,
                                                       'dur': self.t_sim,
                                                       'amp': bin_rs * -1 + amp_rs}
        self.netParams.stimTargetParams['Input_RS->RS'] = {'source': 'Input_RS',
                                                           'conds': {'pop': 'CTX_RS'},
                                                           'sec': 'rs',
                                                           'loc': 0}

        # GPe receve a constante 3 density current or 1 during cortical stimulation
        self.netParams.stimSourceParams['Input_GPe'] = {'type': 'IClamp',
                                                        'delay': 0,
                                                        'dur': self.t_sim,
                                                        'amp': bin_gpe * -1 + amp_gpe}
        self.netParams.stimTargetParams['Input_GPe->GPe'] = {'source': 'Input_GPe',
                                                             'conds': {'pop': 'GPe'},
                                                             'sec': 'GPe',
                                                             'loc': 0}

        # GPi receve a constante 3 density current
        self.netParams.stimSourceParams['Input_GPi'] = {'type': 'IClamp',
                                                        'delay': 0, 'dur': self.t_sim,
                                                        'amp': bin_gpi * -1 + amp_gpi}
        self.netParams.stimTargetParams['Input_GPi->GPi'] = {'source': 'Input_GPi',
                                                             'conds': {'pop': 'GPi'},
                                                             'sec': 'GPi',
                                                             'loc': 0}

        # STN receve a constante 3 density current or 1 during cortical stimulation
        self.netParams.stimSourceParams['Input_STN'] = {'type': 'IClamp',
                                                        'delay': 0,
                                                        'dur': self.t_sim,
                                                        'amp': bin_stn * -1 + amp_stn}
        self.netParams.stimTargetParams['Input_STN->STN'] = {'source': 'Input_STN',
                                                             'conds': {'pop': 'STN'},
                                                             'sec': 'STN',
                                                             'loc': 0}

        # dStr receve a constante 3 density current
        self.netParams.stimSourceParams['Input_StrD1'] = {'type': 'IClamp',
                                                          'delay': 0,
                                                          'dur': self.t_sim,
                                                          'amp': bin_dstr * -1 + amp_dstr}
        self.netParams.stimTargetParams['Input_StrD1->StrD1'] = {'source': 'Input_StrD1',
                                                                 'conds': {'pop': 'StrD1'},
                                                                 'sec': 'StrD1',
                                                                 'loc': 0}

        # iStr receve a constante 3 density current
        self.netParams.stimSourceParams['Input_StrD2'] = {'type': 'IClamp',
                                                          'delay': 0, 'dur': self.t_sim,
                                                          'amp': bin_istr * -1 + amp_istr}
        self.netParams.stimTargetParams['Input_StrD2->StrD2'] = {'source': 'Input_StrD2',
                                                                 'conds': {'pop': 'StrD2'},
                                                                 'sec': 'StrD2',
                                                                 'loc': 0}

        # Thalamus receve a constante 1.2 density current or 1.8
        print("amplitude received: ",self.amplitude)
        self.netParams.stimSourceParams['Input_th'] = {'type': 'IClamp',
                                                       'delay': 0,
                                                       'dur': self.t_sim,
                                                       'amp': self.amplitude}#bin_istr * -1 + amp_istr}
        self.netParams.stimTargetParams['Input_th->TH'] = {'source': 'Input_th',
                                                           'conds': {'pop': 'TH'},
                                                           'sec': 'th',
                                                           'loc': 0}
        
    def buildSimConfig(self, dt=0.1, lfp=False):
        # Simulation parameters
        simConfig = specs.SimConfig()
        simConfig.duration = self.t_sim  # Duration of the simulation, in ms
        simConfig.dt = dt  # Internal integration timestep to use
        simConfig.verbose = False  # Show detailed messages
        simConfig.printPopAvgRates = True

        # Saving
        simConfig.filename = 'model_output'  # Set file output name
        simConfig.saveFileStep = 1 # step size in ms to save data to disk
        simConfig.savePickle = False # save to pickle file
        simConfig.saveJson = False # save to json file
        simConfig.saveMat = True # save to mat file
        simConfig.saveTxt = True # save to txt file
        simConfig.saveDpk = False # save to .dpk pickled file
        simConfig.saveHDF5 = False # save to HDF5 file

        # Recording
        
        simConfig.recordStep = 1  # Step size in ms to save data (eg. V traces, LFP, etc)
        simConfig.recordCells = ['allCells']
        simConfig.recordSpikesGids = True #list of cells to record spike times from
        

        simConfig.analysis['plotRaster'] = {'saveFig': 'tut6_raster.png'}  # Plot a raster
        simConfig.analysis['plotTraces'] = {'include': [1]}     

        return simConfig

    def simulate(self, has_pd=1, dt=0.1, lfp=False, interval = 1000):
        # Config
        simConfig = self.buildSimConfig(dt=dt, lfp=lfp)

        # Output
        if has_pd == 1:
            simConfig.filename = 'model_output_PD'
        else:
            simConfig.filename = 'model_output_H'

        # Init            
        sim.initialize(simConfig=simConfig,netParams=self.netParams)

        sim.net.createPops()
        sim.net.createCells()
        sim.net.connectCells()
        sim.net.addStims()
        sim.setupRecording()

        # Run
        sim.runSimWithIntervalFunc(interval, self.adjustNetworkAndCommunicateViaROS)


        # Save        
        sim.saveData()
        
    def adjustNetworkAndCommunicateViaROS(self,time,):
        # Check time
        self.current_interval = time
        #print("current interval", time)
         
        #we only gather and publish data at the last interval.
        if self.t_sim<= self.current_interval+0.1:
            # Collect Data
            sim.gatherData()
            # Plot a raster
            sim.analysis.plotData()
            #sim.analysis.plotRaster()  
            
            #extract average population rates
            for item, value in sim.allSimData.popRates.items():
                #print("poprates",item,value)
                self.avpopratesregions.append(value)
            #extract global average rate
            #print("averagepop",sim.allSimData.avgRate)

            # Do the analyses
            # ADD CODE HERE

            # Send the new information via ROS
            self.pub.publish(self.avpopratesregions)
            #self.avpopratesregions=[]

if __name__ == '__main__':
    try:
        # ROS
        rospy.init_node('neural_model_rat', anonymous=False)
        # Create network
        pd = 1
        #while not rospy.is_shutdown():
        network = Network(t_sim = 2000, has_pd=pd) #90000
        # Simulate 
        network.simulate(pd, dt = 0.1, lfp = False, interval = 100) #1000
        
    except rospy.ROSInterruptException:
        pass
    
              
