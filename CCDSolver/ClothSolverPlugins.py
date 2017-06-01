import maya.OpenMaya as OpenMaya
import maya.OpenMayaMPx as OpenMayaMPx
#PlugIns
import twClothSolverIOPaperBlockout
import twMeshInfoCmd
import twCheckCollision
import twDrawBB
import sys
#Pymel
import pymel.core as pm
import maya.cmds as cmds

class GUI: 

    GlobalVars = {}
    NumOfCol = 0
    menuBar=pm.menu()
    #def __init__(self):
    #    self.Menubar()
    def CreateCloth(self,*args):
	    #TwClothSolverIOPaperBlockout
	    #----------------------------------
	    #----------------------------------
	
	    #Get Pasing Parameters:
	    subx = pm.intSliderGrp(self.GlobalVars["SubX"], q = True, value = True)
	    suby = pm.intSliderGrp(self.GlobalVars["SubY"], q = True, value = True)
	    grav = pm.floatSliderGrp(self.GlobalVars["GraV"], q = True, value = True)
	    wind_x = pm.floatSliderGrp(self.GlobalVars["Wind_X"], q = True, value = True)
	    wind_y = pm.floatSliderGrp(self.GlobalVars["Wind_Y"], q = True, value = True)
	    wind_z = pm.floatSliderGrp(self.GlobalVars["Wind_Z"], q = True, value = True)
	    noise = pm.floatSliderGrp(self.GlobalVars["Noise"], q = True, value = True)

	    #create inputGeo
	    #----------------------------------
	    #inputGeo
	    inputGeoTrans = pm.polyPlane( sx=subx, sy=suby, w=10, h=10, ch = False)[0]
	    inputGeoShape = inputGeoTrans.getShape()
	    #transformPlane
	    inputGeoTrans.translateY.set(5)
	    inputGeoTrans.visibility.set(0)
	    pm.select(cl = True)
	    #pm.select(inputGeoTrans, r = True)
	    #pm.polyTriangulate(ch = False)
	    #pm.select(cl = True)

	    #create outputGeo
	    #----------------------------------
	    #outputGeo
	    outputGeoTrans = pm.polyPlane( sx=subx, sy=suby, w=10, h=10, ch = False)[0]
	    outputGeoShape = outputGeoTrans.getShape()
	    pm.select(cl = True)
	    #set Texture
	    #if pm.checkBox(GlobalVars["DefaultText"], q = True, value = True) == True:
	    #	pm.sets("lambert2SG", e = True, forceElement = inputGeoTrans)
	
	    #create cloth solver node
	    #----------------------------------
	    clothSolver = pm.createNode('TwClothSolverIOPaperBlockout')
	    clothSolver.verbose.set(0)
	    clothSolver.repetitions.set(10)
	    self.GlobalVars["clothSolver"] = clothSolver
	    pm.select(cl = True)

	    #create positionConstraintLocators
	    #----------------------------------
	    positionConstraintLocatorIndex0Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex0')
	    positionConstraintLocatorIndex0Shape = positionConstraintLocatorIndex0Trans.getShape()
	    positionConstraintLocatorIndex0Trans.translate.set(-5,5,5)
	    pm.select(cl = True)

	    positionConstraintLocatorIndex110Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex110')
	    positionConstraintLocatorIndex110Shape = positionConstraintLocatorIndex110Trans.getShape()
	    positionConstraintLocatorIndex110Trans.translate.set(-5,5,-5)
	    pm.select(cl = True)

	    positionConstraintLocatorIndex120Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex120')
	    positionConstraintLocatorIndex120Shape = positionConstraintLocatorIndex120Trans.getShape()
	    positionConstraintLocatorIndex120Trans.translate.set(5,5,-5)
	    pm.select(cl = True)

	    positionConstraintLocatorIndex10Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex10')
	    positionConstraintLocatorIndex10Shape = positionConstraintLocatorIndex10Trans.getShape()
	    positionConstraintLocatorIndex10Trans.translate.set(5,5,5)
	    pm.select(cl = True)

	    positionConstraintLocatorIndex60Trans = pm.spaceLocator(n = 'positionConstraintLocatorIndex60')
	    positionConstraintLocatorIndex60Shape = positionConstraintLocatorIndex60Trans.getShape()
	    positionConstraintLocatorIndex60Trans.translate.set(0,5,0)
	    pm.select(cl = True)

	    #connect locators and set index attrs on clothSolver
	    #----------------------------------
	    clothSolver.positionConstraint[0].positionConstraintVertexIndex.set(0)
	    positionConstraintLocatorIndex0Shape.worldPosition >> clothSolver.positionConstraint[0].positionConstraintCoordinate
	    pm.select(cl = True)
	    clothSolver.positionConstraint[1].positionConstraintVertexIndex.set(((subx + 1)*(suby + 1) - 1) - subx)
	    positionConstraintLocatorIndex110Shape.worldPosition >> clothSolver.positionConstraint[1].positionConstraintCoordinate
	    pm.select(cl = True)
	    clothSolver.positionConstraint[2].positionConstraintVertexIndex.set((subx + 1)*(suby + 1) - 1)
	    positionConstraintLocatorIndex120Shape.worldPosition >> clothSolver.positionConstraint[2].positionConstraintCoordinate
	    pm.select(cl = True)
	    clothSolver.positionConstraint[3].positionConstraintVertexIndex.set(subx)
	    positionConstraintLocatorIndex10Shape.worldPosition >> clothSolver.positionConstraint[3].positionConstraintCoordinate
	    pm.select(cl = True)
	    clothSolver.positionConstraint[4].positionConstraintVertexIndex.set(((subx + 1)*(suby + 1) - 1)/2)
	    positionConstraintLocatorIndex60Shape.worldPosition >> clothSolver.positionConstraint[4].positionConstraintCoordinate
	    pm.select(cl = True)

	    #set clothSolver active attr
	    #----------------------------------
	    clothSolver.positionConstraint[0].positionConstraintActive.set(pm.checkBox(self.GlobalVars["Locator_LeftTopCorner"], q = True, value = True))
	    clothSolver.positionConstraint[1].positionConstraintActive.set(pm.checkBox(self.GlobalVars["Locator_RightTopCorner"], q = True, value = True))
	    clothSolver.positionConstraint[2].positionConstraintActive.set(pm.checkBox(self.GlobalVars["Locator_RightBottomCorner"], q = True, value = True))
	    clothSolver.positionConstraint[3].positionConstraintActive.set(pm.checkBox(self.GlobalVars["Locator_LeftBottomCorner"], q = True, value = True))
	    clothSolver.positionConstraint[4].positionConstraintActive.set(pm.checkBox(self.GlobalVars["Locator_Middle"], q = True, value = True))
	    #connect time
	    #----------------------------------
	    timeNode = pm.PyNode('time1')
	    pm.select(cl = True)
	    timeNode.outTime >> clothSolver.currentFrame

	    #connect inputGeo
	    #----------------------------------
	    inputGeoShape.outMesh >> clothSolver.inputGeo

	    #connect inputGeo parentMatrix
	    #----------------------------------
	    inputGeoShape.parentMatrix >> clothSolver.transformMatrix

	    #connect outputGeo
	    #----------------------------------
	    clothSolver.outputGeo >> outputGeoShape.inMesh

	    #set clothSolver gravity
	    #----------------------------------
	    gravityPerSec = -1.0 * grav
	    framesPerSec = 400
	    clothSolver.gravity.set(0,gravityPerSec/framesPerSec,0)

	    #set clothSolver WindForce
	    #----------------------------------
	    clothSolver.WindForce.set(wind_x/framesPerSec,wind_y/framesPerSec,wind_z/framesPerSec)
    
        #set clothSolber Noise
        #----------------------------------
	    clothSolver.Noise.set(noise/framesPerSec)

	    #set De
	    #set time range
	    #----------------------------------
	    pm.playbackOptions(ast = 1, aet = 5000, max = 5000, min = 1)

	
	    #rename and select clothSolver
	    #----------------------------------
	    clothSolverTrans = clothSolver.getParent()
	    pm.rename(clothSolver, 'twClothSolverShape')
	    pm.rename(clothSolverTrans, 'twClothSolver')
	    pm.select(clothSolver, r = True)

    def AssignCCDCollision(self,*args):
	    Obj = pm.ls(selection = True)
	    ObjShape = Obj[0].getShape()
	    #connect the shape as a collision object
	    #---------------------------------------------------
	    self.GlobalVars["clothSolver"].collisionConstraint[self.NumOfCol].collisionConstraintActive.set(1)
	    self.GlobalVars["clothSolver"].collisionConstraint[self.NumOfCol].collisionConstraintType.set(1)
	    ObjShape.outMesh >> self.GlobalVars["clothSolver"].collisionConstraint[self.NumOfCol].collisionConstraintGeo
	    ObjShape.parentMatrix >> self.GlobalVars["clothSolver"].collisionConstraint[self.NumOfCol].collisionConstraintGeoMatrix
	    self.NumOfCol = self.NumOfCol + 1
	    pm.select(cl = True)
    
    
    def AssignRegularCollision(self,*args):
	    Obj = pm.ls(selection = True)
	    ObjShape = Obj[0].getShape()
	    #connect the shape as a collision object
	    #---------------------------------------------------
	    self.GlobalVars["clothSolver"].collisionConstraint[self.NumOfCol].collisionConstraintActive.set(1)
	    self.GlobalVars["clothSolver"].collisionConstraint[self.NumOfCol].collisionConstraintType.set(2)
	    ObjShape.outMesh >> self.GlobalVars["clothSolver"].collisionConstraint[self.NumOfCol].collisionConstraintGeo
	    ObjShape.parentMatrix >> self.GlobalVars["clothSolver"].collisionConstraint[self.NumOfCol].collisionConstraintGeoMatrix
	    self.NumOfCol = self.NumOfCol + 1
	    pm.select(cl = True)

    def AddSphere(self,*args):
	    #create sphere
	    #----------------------------------
	    collisionSphereTrans = pm.polySphere(sx = 8,sy = 8, n = 'collisionSphere0', r = 2, ch = 0)[0]
	    collisionSphereShape = collisionSphereTrans.getShape()
	    collisionSphereTrans.translate.set(0,1,0)
	    pm.select(cl = True)

	    #connect sphere as collision object
	    #----------------------------------
	    clothSolver = self.GlobalVars["clothSolver"]
	    clothSolver.collisionConstraint[self.NumOfCol].collisionConstraintActive.set(1)
	    clothSolver.collisionConstraint[self.NumOfCol].collisionConstraintType.set(0)
	    collisionSphereShape.outMesh >> clothSolver.collisionConstraint[self.NumOfCol].collisionConstraintGeo
	    collisionSphereShape.parentMatrix >> clothSolver.collisionConstraint[self.NumOfCol].collisionConstraintGeoMatrix
	    self.NumOfCol = self.NumOfCol + 1
	    pm.select(cl = True)

    def SelectSolver(self,*args):
	    pm.select(self.GlobalVars["clothSolver"], r = True)
        
    def Pr(self,*args):
        #Read Parameters:
	    subx = pm.intSliderGrp(self.GlobalVars["SubX"], q = True, value = True)
	    suby = pm.intSliderGrp(self.GlobalVars["SubY"], q = True, value = True)
	    grav = pm.floatSliderGrp(self.GlobalVars["GraV"], q = True, value = True)
	
	    print subx
	    print suby
	    print grav

    def CreateClothWindow(self,*args):
	    if pm.window("cloth", exists = True):
	        pm.deleteUI("cloth")

	    cloth = pm.window("Create Cloth", t = "Create Cloth", w = 400, h = 600)
	    pm.columnLayout(adj = True, cal = "center", columnAttach=('both', 25))
	    pm.separator(h = 30)
	    pm.text("Cloth Simulation with PBD and exact CCD", font = "boldLabelFont")
	    pm.separator(h = 30)
	    pm.frameLayout(label = "Cloth Subdivisions", collapsable = True, borderStyle = "etchedIn")
	    self.GlobalVars["SubX"] = pm.intSliderGrp(l = "Subdivision X", min = 1, max = 20, value = 10, field = True)
	    self.GlobalVars["SubY"] = pm.intSliderGrp(l = "Subdivision Y", min = 1, max = 20, value = 10, field = True)
	    pm.setParent("..")
	    pm.frameLayout(label = "Position Constraints", collapsable = True, borderStyle = "etchedIn")
	    pm.gridLayout(numberOfColumns = 2, cellWidthHeight = (180,20))
	    self.GlobalVars["Locator_LeftTopCorner"] = pm.checkBox(label = "Locator_LeftTopCorner")
	    self.GlobalVars["Locator_RightTopCorner"] = pm.checkBox(label = "Locator_RightTopCorner")
	    self.GlobalVars["Locator_RightBottomCorner"] = pm.checkBox(label = "Locator_RightBottomCorner")
	    self.GlobalVars["Locator_LeftBottomCorner"] = pm.checkBox(label = "Locator_LeftBottomCorner")
	    self.GlobalVars["Locator_Middle"] = pm.checkBox(label = "Locator_Middle")
	    pm.setParent("..")
	    pm.setParent("..")
	    pm.frameLayout(label = "Gravity", collapsable = True, borderStyle = "etchedIn")
	    self.GlobalVars["GraV"] = pm.floatSliderGrp(l = "Gravity", min = 0.0, max = 9.8, field = True)
	    pm.setParent("..")
	    pm.frameLayout(label = "Wind Force", collapsable = True, borderStyle = "etchedIn")
	    self.GlobalVars["Wind_X"] = pm.floatSliderGrp(l = "Wind Force_X", min = -10.0, max = 10, value = 0.0, field = True)
	    self.GlobalVars["Wind_Y"] = pm.floatSliderGrp(l = "Wind Force_Y", min = -10.0, max = 10, value = 0.0, field = True)
	    self.GlobalVars["Wind_Z"] = pm.floatSliderGrp(l = "Wind Force_Z", min = -10.0, max = 10, value = 0.0, field = True)
	    self.GlobalVars["Noise"] = pm.floatSliderGrp(l = "Noise", min = 0.0, max = 2, value = 0.0, field = True)
	    pm.setParent("..")
	    pm.separator(h = 10)
	    self.GlobalVars["DefaultText"] = pm.checkBox(label = "Default Texture")
	    pm.separator(h = 10)
	    pm.button(l = "Create", c = self.CreateCloth)
	    pm.showWindow(cloth)

    def test(self,*args):
        OpenMaya.MGlobal.displayInfo('Connected')
    def DeleteMenubar(self,*args):
        cmds.deleteUI(self.menuBar,menu=True)
    def Menubar(self,*args):
	    self.menuBar = pm.menu(parent = 'MayaWindow', label = "Cloth Simulation CIS660", tearOff = True)
            pm.menuItem(label = "Create Cloth", command = self.CreateClothWindow)
            pm.menuItem(label = "Assign the selected objects as Collision Objects with CCD", command = self.AssignCCDCollision)
            pm.menuItem(label = "Assign the selected objects as Collision Objects with regular method", command = self.AssignRegularCollision)
            pm.menuItem(label = "Add a Sphere Collision object(basic method)", command = self.AddSphere)
            pm.menuItem(label = "Select the Solver", command = self.SelectSolver)
        

gui=GUI()
def initializePlugin(mobject):
    '''Register Plugin'''
    mplugin = OpenMayaMPx.MFnPlugin(mobject)  
    try:
        mplugin.registerCommand('MeshInfo', twMeshInfoCmd.createTwMeshInfoCmd)
        mplugin.registerNode( twCheckCollision.TwCheckCollision.kPluginNodeName, twCheckCollision.TwCheckCollision.kPluginNodeId, twCheckCollision.createTwCheckCollision, twCheckCollision.initializeTwCheckCollision, OpenMayaMPx.MPxNode.kLocatorNode)
        mplugin.registerNode( twDrawBB.TwDrawBB.kPluginNodeName, twDrawBB.TwDrawBB.kPluginNodeId, twDrawBB.createTwDrawBB, twDrawBB.initializeTwDrawBB, OpenMayaMPx.MPxNode.kLocatorNode)
        mplugin.registerNode( twClothSolverIOPaperBlockout.TwClothSolverIOPaperBlockout.kPluginNodeName, twClothSolverIOPaperBlockout.TwClothSolverIOPaperBlockout.kPluginNodeId, twClothSolverIOPaperBlockout.createTwClothSolverIOPaperBlockout, twClothSolverIOPaperBlockout.initializeTwClothSolverIOPaperBlockout, OpenMayaMPx.MPxNode.kLocatorNode)
        print('Successfully loaded plugin')
        gui.Menubar()
    except:
        sys.stderr.write( "Failed to register node: %s" % kPluginNodeName )
        raise
        
    
    


def uninitializePlugin(mobject):
    '''Deregister Plugin'''
    
    mplugin = OpenMayaMPx.MFnPlugin(mobject)
    try:
        mplugin.deregisterCommand('MeshInfo')
        mplugin.deregisterNode( twCheckCollision.TwCheckCollision.kPluginNodeId )
        mplugin.deregisterNode( twDrawBB.TwDrawBB.kPluginNodeId )
        mplugin.deregisterNode( twClothSolverIOPaperBlockout.TwClothSolverIOPaperBlockout.kPluginNodeId )
        gui.DeleteMenubar()
    except:
        sys.stderr.write( "Failed to deregister node: %s" % kPluginNodeName )
        raise
    
    
    