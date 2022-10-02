# -*- coding: mbcs -*-
# Do not delete the following import lines
from abaqus import *
from abaqusConstants import *
import __main__
import section
import regionToolset
import displayGroupMdbToolset as dgm
import part
import material
import assembly
import step
import interaction
import load
import mesh
import optimization
import job
import sketch
import visualization
import xyPlot
import displayGroupOdbToolset as dgo
import connectorBehavior
import numpy as np
from abaqusConstants import *
from caeModules import *
from abaqus import *
from odbAccess import *

index = np.genfromtxt('index.csv', dtype=float, delimiter=',')
index = int(index)

for j in range (10):
    c = []
    c_data = np.genfromtxt('DesignVariables.csv', dtype=float, delimiter=',')
    for i in range (20):
        c.append(c_data[index - 1, i])
    
    L0 = 6
    c1 = c[2 * j]
    c2 = c[2 * j + 1]
    a = np.arange(-(np.pi), np.pi, 0.05)
    r0 = L0/(np.pi*(2+c1**2+c2**2))**0.5
    r = r0*(1+c1*np.cos(4*a)+c2*np.cos(8*a))
    x = r*np.cos(a)
    y = r*np.sin(a)
    s = mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=20.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.rectangle(point1=(-5.0, -5.0), point2=(5.0, 5.0))
    point1=()
    for i in range(len(x)):
        point1 = point1+((x[i],y[i]),)
    
    point1 = point1 + ((x[0], y[0]),)
    s.Spline(points=point1)
    
#    p = mdb.models['Model-1'].Part(name='Part-'+str(j), dimensionality=THREE_D,
#        type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].Part(name='Part-'+str(j), dimensionality=TWO_D_PLANAR, 
        type=DEFORMABLE_BODY)
    p = mdb.models['Model-1'].parts['Part-'+str(j)]
    p.BaseShell(sketch=s)
    s.unsetPrimaryObject()
    p = mdb.models['Model-1'].parts['Part-'+str(j)]
    session.viewports['Viewport: 1'].setValues(displayedObject=p)
    del mdb.models['Model-1'].sketches['__profile__']


#assembly
a1 = mdb.models['Model-1'].rootAssembly
for i in range (10):
    p = mdb.models['Model-1'].parts['Part-'+str(i)]
    a1.Instance(name='Part-'+str(i)+'-1', part=p, dependent=ON)

for i in range (1,10):
    p1 = a1.instances['Part-'+str(i)+'-1']
    p1.translate(vector=(10*i, 0.0, 0.0))

a1.InstanceFromBooleanMerge(name='Part-10', instances=(
    a1.instances['Part-0-1'], a1.instances['Part-1-1'], 
    a1.instances['Part-2-1'], a1.instances['Part-3-1'], 
    a1.instances['Part-4-1'], a1.instances['Part-5-1'], 
    a1.instances['Part-6-1'], a1.instances['Part-7-1'], 
    a1.instances['Part-8-1'], a1.instances['Part-9-1'], ), 
    originalInstances=SUPPRESS, domain=GEOMETRY)
session.viewports['Viewport: 1'].assemblyDisplay.setValues(
    adaptiveMeshConstraints=ON)
    

#MATERIALS
mdb.models['Model-1'].Material(name='Material-1')
mdb.models['Model-1'].materials['Material-1'].Hyperelastic(
    materialType=ISOTROPIC, testData=OFF, type=NEO_HOOKE,
    volumetricResponse=VOLUMETRIC_DATA, table=((0.5, 0.0), ))
mdb.models['Model-1'].HomogeneousSolidSection(name='Section-1',
                                              material='Material-1', thickness=0.2)
p = mdb.models['Model-1'].parts['Part-10']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#1 ]',), )

region = p.Set(faces=faces, name='Set-1')
p = mdb.models['Model-1'].parts['Part-10']
p.SectionAssignment(region=region, sectionName='Section-1', offset=0.0,
                    offsetType=MIDDLE_SURFACE, offsetField='',
                    thicknessAssignment=FROM_SECTION)
a1 = mdb.models['Model-1'].rootAssembly
a1.regenerate()
a = mdb.models['Model-1'].rootAssembly
session.viewports['Viewport: 1'].setValues(displayedObject=a)

mdb.models['Model-1'].BuckleStep(name='Step-1', previous='Initial',
                                 numEigen=20, vectors=28, maxIterations=800)

a = mdb.models['Model-1'].rootAssembly
a.ReferencePoint(point=(110.0, 0.0, 0.0))
a = mdb.models['Model-1'].rootAssembly
r1 = a.referencePoints
refPoints1 = (r1[23],)
region1 = a.Set(referencePoints=refPoints1, name='m_Set-3')
a = mdb.models['Model-1'].rootAssembly
e1 = a.instances['Part-10-1'].edges
edges1 = e1.getSequenceFromMask(mask=('[#100000 ]',), )
region2 = a.Set(edges=edges1, name='s_Set-3')
mdb.models['Model-1'].Coupling(name='Constraint-1', controlPoint=region1,
                               surface=region2, influenceRadius=WHOLE_SURFACE, couplingType=KINEMATIC,
                               localCsys=None, u1=ON, u2=ON, ur3=ON)

a = mdb.models['Model-1'].rootAssembly
e1 = a.instances['Part-10-1'].edges
edges1 = e1.getSequenceFromMask(mask=('[#80000000 ]',), )
region = a.Set(edges=edges1, name='Set-1')
mdb.models['Model-1'].DisplacementBC(name='BC-1', createStepName='Initial',
                                     region=region, u1=SET, u2=SET, ur3=SET, amplitude=UNSET,
                                     distributionType=UNIFORM, fieldName='', localCsys=None)

a = mdb.models['Model-1'].rootAssembly
e1 = a.instances['Part-10-1'].edges
edges1 = e1.getSequenceFromMask(mask=('[#100000 ]',), )
region = a.Set(edges=edges1, name='Set-2')
mdb.models['Model-1'].DisplacementBC(name='BC-2', createStepName='Initial',
                                     region=region, u1=UNSET, u2=SET, ur3=SET, amplitude=UNSET,
                                     distributionType=UNIFORM, fieldName='', localCsys=None)

r1 = a.referencePoints
refPoints1 = (r1[23],)
region = a.Set(referencePoints=refPoints1, name='Set-5')
mdb.models['Model-1'].DisplacementBC(name='BC-3', createStepName='Initial',
                                     region=region, u1=UNSET, u2=SET, ur3=SET, amplitude=UNSET,
                                     distributionType=UNIFORM, fieldName='', localCsys=None)
r1 = a.referencePoints
refPoints1 = (r1[23],)
a = mdb.models['Model-1'].rootAssembly
region = a.Set(referencePoints=refPoints1, name='Set-6')
mdb.models['Model-1'].ConcentratedForce(name='Load-1', createStepName='Step-1',
                                        region=region, cf1=-1.0, distributionType=UNIFORM, field='',
                                        localCsys=None)


elemType1 = mesh.ElemType(elemCode=CPE8R, elemLibrary=STANDARD)
elemType2 = mesh.ElemType(elemCode=CPE6H, elemLibrary=STANDARD)
p = mdb.models['Model-1'].parts['Part-10']
f = p.faces
faces = f.getSequenceFromMask(mask=('[#1 ]',), )
pickedRegions = (faces,)
p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2))
p = mdb.models['Model-1'].parts['Part-10']
f = p.faces
pickedRegions = f.getSequenceFromMask(mask=('[#1 ]',), )
p.setMeshControls(regions=pickedRegions, elemShape=TRI)
p = mdb.models['Model-1'].parts['Part-10']
p.seedPart(size=0.5, deviationFactor=0.1, minSizeFactor=0.1)
p = mdb.models['Model-1'].parts['Part-10']
p.generateMesh()


mdb.Job(atTime=None, contactPrint=OFF, description='', echoPrint=OFF,
    explicitPrecision=SINGLE, getMemoryFromAnalysis=True, historyPrint=OFF,
    memory=90, memoryUnits=PERCENTAGE, model='Model-1', modelPrint=OFF,
    multiprocessingMode=DEFAULT, name='job-1', nodalOutputPrecision=SINGLE,
    numCpus=2, numDomains=2, numGPUs=0, queue=None, resultsFormat=ODB,
    scratch='', type=ANALYSIS, userSubroutine='', waitHours=0, waitMinutes=0)
mdb.jobs['job-1'].submit(consistencyChecking=OFF)
mdb.jobs['job-1'].waitForCompletion()


o1 = session.openOdb(name='job-1.odb')
session.viewports['Viewport: 1'].setValues(displayedObject=o1)


numEigenvalue = 1
myOdb = visualization.openOdb(path='job-1.odb')
for framesEigen in range(1, 21):
    if myOdb.steps['Step-1'].frames[-framesEigen].mode==numEigenvalue:
        eigenValueDescription=myOdb.steps['Step-1'].frames[-framesEigen].description
        eigenValueLoc=eigenValueDescription.index('=')+1
        eigenValue1=(float(eigenValueDescription[eigenValueLoc:]))
        break


with open('eigenValue4000.csv', 'a') as file_object:
    file_object.write(str(index) + ',')
    file_object.write(str(eigenValue1))
    file_object.write('\n')


session.odbs['job-1.odb'].close()
index = index + 1
index = int(index)
with open('index.csv', 'w') as file_object:
    file_object.write(str(index))

