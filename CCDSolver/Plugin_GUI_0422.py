import pymel.core as pm
import maya.cmds as cmds

#Global Variables:
#SubX, SubY, GraV, clothSolver
GlobalVars = {}
NumOfCol = 0;

def CreateCloth():
	#TwClothSolverIOPaperBlockout
	#----------------------------------
	#----------------------------------
	
	#Get Pasing Parameters:
	subx = pm.intSliderGrp(GlobalVars["SubX"], q = True, value = True)
	suby = pm.intSliderGrp(GlobalVars["SubY"], q = True, value = True)
	grav = pm.floatSliderGrp(GlobalVars["GraV"], q = True, value = True)
	wind_x = pm.floatSliderGrp(GlobalVars["Wind_X"], q = True, value = True)
	wind_y = pm.floatSliderGrp(GlobalVars["Wind_Y"], q = True, value = True)
	wind_z = pm.floatSliderGrp(GlobalVars["Wind_Z"], q = True, value = True)
	noise = pm.floatSliderGrp(GlobalVars["Noise"], q = True, value = True)

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
	GlobalVars["clothSolver"] = clothSolver
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
	clothSolver.positionConstraint[0].positionConstraintActive.set(pm.checkBox(GlobalVars["Locator_LeftTopCorner"], q = True, value = True))
	clothSolver.positionConstraint[1].positionConstraintActive.set(pm.checkBox(GlobalVars["Locator_RightTopCorner"], q = True, value = True))
	clothSolver.positionConstraint[2].positionConstraintActive.set(pm.checkBox(GlobalVars["Locator_RightBottomCorner"], q = True, value = True))
	clothSolver.positionConstraint[3].positionConstraintActive.set(pm.checkBox(GlobalVars["Locator_LeftBottomCorner"], q = True, value = True))
	clothSolver.positionConstraint[4].positionConstraintActive.set(pm.checkBox(GlobalVars["Locator_Middle"], q = True, value = True))
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


def AssignCCDCollision():
	global NumOfCol
	Obj = pm.ls(selection = True)
	ObjShape = Obj[0].getShape()
	#connect the shape as a collision object
	#---------------------------------------------------
	GlobalVars["clothSolver"].collisionConstraint[NumOfCol].collisionConstraintActive.set(1)
	GlobalVars["clothSolver"].collisionConstraint[NumOfCol].collisionConstraintType.set(1)
	ObjShape.outMesh >> GlobalVars["clothSolver"].collisionConstraint[NumOfCol].collisionConstraintGeo
	ObjShape.parentMatrix >> GlobalVars["clothSolver"].collisionConstraint[NumOfCol].collisionConstraintGeoMatrix
	NumOfCol = NumOfCol + 1
	pm.select(cl = True)

def AssignRegularCollision():
	global NumOfCol
	Obj = pm.ls(selection = True)
	ObjShape = Obj[0].getShape()
	#connect the shape as a collision object
	#---------------------------------------------------
	GlobalVars["clothSolver"].collisionConstraint[NumOfCol].collisionConstraintActive.set(1)
	GlobalVars["clothSolver"].collisionConstraint[NumOfCol].collisionConstraintType.set(2)
	ObjShape.outMesh >> GlobalVars["clothSolver"].collisionConstraint[NumOfCol].collisionConstraintGeo
	ObjShape.parentMatrix >> GlobalVars["clothSolver"].collisionConstraint[NumOfCol].collisionConstraintGeoMatrix
	NumOfCol = NumOfCol + 1
	pm.select(cl = True)

def AddSphere():
	global NumOfCol
	#create sphere
	#----------------------------------
	collisionSphereTrans = pm.polySphere(sx = 8,sy = 8, n = 'collisionSphere0', r = 2, ch = 0)[0]
	collisionSphereShape = collisionSphereTrans.getShape()
	collisionSphereTrans.translate.set(0,1,0)
	pm.select(cl = True)

	#connect sphere as collision object
	#----------------------------------
	clothSolver = GlobalVars["clothSolver"]
	clothSolver.collisionConstraint[NumOfCol].collisionConstraintActive.set(1)
	clothSolver.collisionConstraint[NumOfCol].collisionConstraintType.set(0)
	collisionSphereShape.outMesh >> clothSolver.collisionConstraint[NumOfCol].collisionConstraintGeo
	collisionSphereShape.parentMatrix >> clothSolver.collisionConstraint[NumOfCol].collisionConstraintGeoMatrix
	NumOfCol = NumOfCol + 1
	pm.select(cl = True)

def SelectSolver():
	pm.select(GlobalVars["clothSolver"], r = True)

def Pr():
    #Read Parameters:
	subx = pm.intSliderGrp(GlobalVars["SubX"], q = True, value = True)
	suby = pm.intSliderGrp(GlobalVars["SubY"], q = True, value = True)
	grav = pm.floatSliderGrp(GlobalVars["GraV"], q = True, value = True)
	
	print subx
	print suby
	print grav

def CreateClothWindow():
	if pm.window("cloth", exists = True):
	    pm.deleteUI("cloth")

	cloth = pm.window("Create Cloth", t = "Create Cloth", w = 400, h = 600)
	pm.columnLayout(adj = True, cal = "center", columnAttach=('both', 25))
	pm.separator(h = 30)
	pm.text("Cloth Simulation with PBD and exact CCD", font = "boldLabelFont")
	pm.separator(h = 30)
	pm.frameLayout(label = "Cloth Subdivisions", collapsable = True, borderStyle = "etchedIn")
	GlobalVars["SubX"] = pm.intSliderGrp(l = "Subdivision X", min = 1, max = 20, value = 10, field = True)
	GlobalVars["SubY"] = pm.intSliderGrp(l = "Subdivision Y", min = 1, max = 20, value = 10, field = True)
	pm.setParent("..")
	pm.frameLayout(label = "Position Constraints", collapsable = True, borderStyle = "etchedIn")
	pm.gridLayout(numberOfColumns = 2, cellWidthHeight = (180,20))
	GlobalVars["Locator_LeftTopCorner"] = pm.checkBox(label = "Locator_LeftTopCorner")
	GlobalVars["Locator_RightTopCorner"] = pm.checkBox(label = "Locator_RightTopCorner")
	GlobalVars["Locator_RightBottomCorner"] = pm.checkBox(label = "Locator_RightBottomCorner")
	GlobalVars["Locator_LeftBottomCorner"] = pm.checkBox(label = "Locator_LeftBottomCorner")
	GlobalVars["Locator_Middle"] = pm.checkBox(label = "Locator_Middle")
	pm.setParent("..")
	pm.setParent("..")
	pm.frameLayout(label = "Gravity", collapsable = True, borderStyle = "etchedIn")
	GlobalVars["GraV"] = pm.floatSliderGrp(l = "Gravity", min = 0.0, max = 9.8, field = True)
	pm.setParent("..")
	pm.frameLayout(label = "Wind Force", collapsable = True, borderStyle = "etchedIn")
	GlobalVars["Wind_X"] = pm.floatSliderGrp(l = "Wind Force_X", min = -10.0, max = 10, value = 0.0, field = True)
	GlobalVars["Wind_Y"] = pm.floatSliderGrp(l = "Wind Force_Y", min = -10.0, max = 10, value = 0.0, field = True)
	GlobalVars["Wind_Z"] = pm.floatSliderGrp(l = "Wind Force_Z", min = -10.0, max = 10, value = 0.0, field = True)
	GlobalVars["Noise"] = pm.floatSliderGrp(l = "Noise", min = 0.0, max = 2, value = 0.0, field = True)
	pm.setParent("..")
	pm.separator(h = 10)
	GlobalVars["DefaultText"] = pm.checkBox(label = "Default Texture")
	pm.separator(h = 10)
	pm.button(l = "Create", c = "CreateCloth()")
	pm.showWindow(cloth)


def Menubar():
	menuBar = pm.menu(parent = 'MayaWindow', label = "Cloth Simulation CIS660", tearOff = True)
        pm.menuItem(label = "Create Cloth", command = "CreateClothWindow()")
        pm.menuItem(label = "Assign the selected objects as Collision Objects with CCD", command = "AssignCCDCollision()")
        pm.menuItem(label = "Assign the selected objects as Collision Objects with regular method", command = "AssignRegularCollision()")
        pm.menuItem(label = "Add a Sphere Collision object(basic method)", command = "AddSphere()")
        pm.menuItem(label = "Select the Solver", command = "SelectSolver()")
Menubar()
