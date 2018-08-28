import ScriptEnv

ScriptEnv.Initialize("Ansoft.ElectronicsDesktop")
oDesktop.RestoreWindow()
oProject = oDesktop.SetActiveProject("catch_tomo")
oDesign = oProject.SetActiveDesign("First_Design")
oEditor = oDesign.SetActiveEditor("3D Modeler")
oModule = oDesign.GetModule("BoundarySetup")

TOP = [0, 1]
DOWN = [0, -1]
RIGHT = [1, 0]
LEFT = [-1, 0]

POS = 0
ORI = 1
TRACK = 2
GAP = 3

trackObjects, gapObjects = [], []

    # def set_variable(self, name, value, postprocessing=False):
        #TODO: check if variable does not exist and quit if it doesn't?
        # if name not in self._design.GetVariables()+self._design.GetPostProcessingVariables():
            # self.create_variable(name, value, postprocessing=postprocessing)
        # else:
            # self._design.SetVariableValue(name, value)
        # return VariableString(name)

    # def get_variable_value(self, name):
        # return self._design.GetVariableValue(name)
        
    # def get_variable_names(self):
        # return [VariableString(s) for s in self._design.GetVariables()+self._design.GetPostProcessingVariables()]  

def assigneInduc(iName, iIn, iOut, iVal):
	oModule.AssignLumpedRLC(
		[
			"NAME:LumpRLC_"+iName,
			"Objects:="		, [iName],
			[
				"NAME:CurrentLine",
				"Start:="		, [str(float(iIn.x))+"mm",str(float(iIn.y))+"mm","0mm"],
				"End:="			, [str(float(iOut.x))+"mm",str(float(iOut.y))+"mm","0mm"]
			],
			"UseResist:="		, False,
			"UseInduct:="		, True,
			"Inductance:="		, "("+str(iVal)+")nH",
			"UseCap:="		, False
		])

def filet(iObject, iFiletRay, iIndex):

	vertices = oEditor.GetVertexIDsFromObject(iObject)
	
	oEditor.Fillet(
	[
		"NAME:Selections",
		"Selections:="		, iObject,
		"NewPartsModelFlag:="	, "Model"
	], 
	[
		"NAME:Parameters",
		[
			"NAME:FilletParameters",
			"Edges:="		, [],
			"Vertices:="		, [int(vertices[iIndex])],
			"Radius:="		, "("+str(iFiletRay)+")mm",
			"Setback:="		, "0mm"
		]
	])
	return

def fuse(iObjects):

	for i in range(len(iObjects)-1):
		oEditor.Unite(
		[
			"NAME:Selections",
			"Selections:="		, iObjects[0]+","+iObjects[i+1]
		], 
		[
			"NAME:UniteParameters",
			"KeepOriginals:="	, False
		])
	
	return iObjects[0]

def assignPerfE(iObject, iName):
	oModule.AssignPerfectE(
	[
		"NAME:PerfE_"+iName,
		"Objects:="		, [iObject],
		"InfGroundPlane:="	, False
	])
	return

def substract(iObecjtTool, iObjectBlanc):

	oEditor.Subtract(
	[
		"NAME:Selections",
		"Blank Parts:="		, iObjectBlanc,
		"Tool Parts:="		, iObecjtTool
	], 
	[
		"NAME:SubtractParameters",
		"KeepOriginals:="	, False
	])
	
	return iObecjtTool

def draw(iName, iPoints):

	pointsStr = ["NAME:PolylinePoints"]
	indexsStr = ["NAME:PolylineSegments"]
	for i in range(len(iPoints)):
		pointsStr.append(["NAME:PLPoint", "X:=", "("+str(iPoints[i].x)+")mm", "Y:=", "("+str(iPoints[i].y)+")mm", "Z:=", "0mm"])
		if(i!=len(iPoints)-1):
			indexsStr.append(["NAME:PLSegment", "SegmentType:=", "Line", "StartIndex:=", i, "NoOfPoints:=", 2])

	oEditor.CreatePolyline(
		[
			"NAME:PolylineParameters",
			"IsPolylineCovered:="	, True,
			"IsPolylineClosed:="	, True,
			pointsStr,	
			indexsStr,
			[
				"NAME:PolylineXSection",
				"XSectionType:="	, "None",
				"XSectionOrient:="	, "Auto",
				"XSectionWidth:="	, "0mm",
				"XSectionTopWidth:="	, "0mm",
				"XSectionHeight:="	, "0mm",
				"XSectionNumSegments:="	, "0",
				"XSectionBendType:="	, "Corner"
			]
		],
		[
			"NAME:Attributes",
			"Name:="		, iName,
			"Flags:="		, "",
			"Color:="		, "(143 175 143)",
			"Transparency:="	, 0,
			"PartCoordinateSystem:=", "Global",
			"UDMId:="		, "",
			"MaterialValue:="	, "\"vacuum\"",
			"SurfaceMaterialValue:=", "\"\"",
			"SolveInside:="		, True,
			"IsMaterialEditable:="	, True,
			"UseMaterialAppearance:=", False
		])
		
	return iName

class LitExp(object):
	val = 0
	exp = ''
	
	def __init__(self, iName='', iDef=0):
		self.val=iDef
		self.exp=str(iName)
		
	def __add__(self, other):
		if(type(other) is LitExp):
			ret = self.copy()
			ret.val += other.val
			ret.exp += " + " + other.exp
			return ret
		else:
			ret = self.copy()
			ret.val += other
			ret.exp += " + " + str(other)
			return ret
		return "This is not possible"

	def __sub__(self, other):
		if(type(other) is LitExp):
			ret = self.copy()
			ret.val -= other.val
			ret.exp += " - (" + other.exp + ")"
			return ret
		else:
			ret = self.copy()
			ret.val -= other
			ret.exp += " - (" + str(other) + ")"
			return ret
		return "This is not possible"

	def __mul__(self, other):
		if(type(other) is LitExp):
			ret = self.copy()
			ret.val *= other.val
			ret.exp = "(" + ret.exp + ")*(" + other.exp + ")"
			return ret
		elif(type(other) is Vector):
			return other*self
		else:
			ret = self.copy()
			ret.val *= other
			ret.exp = "(" + ret.exp + ")*(" + str(other) + ")"
			return ret
		return "This is not possible"
	
	def __div__(self, other):
		if(type(other) is LitExp):
			ret = self.copy()
			ret.val /= other.val
			ret.exp = "(" + ret.exp + ")/(" + other.exp + ")"
			return ret
		elif(type(other) is Vector):
			return other/self
		else:
			ret = self.copy()
			ret.val /= other
			ret.exp = "(" + ret.exp + ")/(" + str(other) + ")"
			return ret
		return "This is not possible"


	def __radd__(self, other):
		return self+other
	def __rsub__(self, other):
		return self-other
	def __rmul__(self, other):
		return self*other
	def __rdiv__(self, other):
		return self/other

	def __neg__(self):
		ret = self.copy()
		ret.val *= -1
		ret.exp = " - (" + ret.exp + ")"
		return ret

	def __lt__(self, other):
		return self.val < other
	def __le__(self, other):
		return self.val <= other
	def __eq__(self, other):
		return self.val == other
	def __ne__(self, other):
		return self.val != other
	def __gt__(self, other):
		return self.val > other
	def __ge__(self, other):
		return self.val >= other
	
	def __abs__(self):
		ret = LitExp()
		ret.val = abs(self.val)
		ret.exp = "abs(" + self.exp + ")"
		return ret

	def __str__(self):
		return self.exp
	def __float__(self):
		return self.val

	def copy(self):
		ret = LitExp()
		ret.val = self.val
		ret.exp = ""+self.exp
		return ret	

class Vector(object):
    x = 0
    y = 0
    def __init__(self, iX, iY=0):
        if(type(iX) is Vector):
            self.x, self.y = iX.x, iX.y
        elif(type(iX) is list):
            self.x, self.y = iX[0], iX[1]
        else:
            self.x, self.y = iX, iY

    def __add__(self, other):
        if(type(other) is list):
            return Vector(self.x+other[0], self.y+other[1])
        elif(type(other) is Vector):
            return Vector(self.x+other.x, self.y+other.y)
        else:
            return Vector(self.x+other, self.y+other)
        return "This is not possible"

    def __sub__(self, other):
        if(type(other) is list):
            return Vector(self.x-other[0], self.y-other[1])
        elif(type(other) is Vector):
            return Vector(self.x-other.x, self.y-other.y)
        else:
            return Vector(self.x-other, self.y-other)
        return "This is not possible"

    def __mul__(self, other):
        if(type(other) is list):
            return self.x*other[0]+self.y*other[1]
        elif(type(other) is Vector):
            return self.x*other.x+self.y*other.y
        else:
            return Vector(self.x*other, self.y*other)
        return "This is not possible"

    def __radd__(self, other):
        return self+other

    def __rsub__(self, other):
        return self-other

    def __rmul__(self, other):
        return self*other

    def __neg__(self):
        return Vector(-self.x, -self.y)

    def __str__(self):
        return "["+str(self.x)+","+str(self.y)+"]"

    def norm(self):
        return (self.x**2+self.y**2)**0.5

    def abs(self):
        return Vector(abs(self.x), abs(self.y))

    def unit(self):
        norm = self.norm()
        return Vector(self.x/norm, self.y/norm)

    def orth(self):
        return Vector(self.y, -self.x)

    def rot(self, other):
        unitOther = other.unit()
        return Vector(self*unitOther, self*unitOther.orth())

def drawCapa(iName, iIn, iOut, iLength, iWidth, iSize):

	pos, ori = Vector(iIn[POS]), Vector(iIn[ORI])
	_, _, inTrack, inGap = iIn
	_, _, outTrack, outGap = iOut
	retIn = [pos, -ori, inTrack, inGap]
	retOut = [pos+(inGap+outGap+iSize+2*iWidth)*ori, ori, outTrack, outGap]

	points = [pos + Vector(inGap+iWidth, 0).rot(ori)]
	points.append(points[-1] + Vector(0, -iLength/2).rot(ori))
	points.append(points[-1] + Vector(-iWidth, 0).rot(ori))
	points.append(points[-1] + Vector(0, iLength/2-inTrack/2).rot(ori))
	points.append(points[-1] + Vector(-inGap, 0).rot(ori))
	points.append(points[-1] + Vector(0, inTrack).rot(ori))
	points.append(points[-1] + Vector(inGap, 0).rot(ori))
	points.append(points[-1] + Vector(0, iLength/2-inTrack/2).rot(ori))
	points.append(points[-1] + Vector(iWidth, 0).rot(ori))
	points.append(points[0])
	trackObjects.append(draw(iName+"_track1", points))

	points = [pos + Vector(inGap+iWidth+iSize, 0).rot(ori)]
	points.append(points[-1] + Vector(0, -iLength/2).rot(ori))
	points.append(points[-1] + Vector(+iWidth, 0).rot(ori))
	points.append(points[-1] + Vector(0, iLength/2-outTrack/2).rot(ori))
	points.append(points[-1] + Vector(+outGap, 0).rot(ori))
	points.append(points[-1] + Vector(0, outTrack).rot(ori))
	points.append(points[-1] + Vector(-outGap, 0).rot(ori))
	points.append(points[-1] + Vector(0, iLength/2-outTrack/2).rot(ori))
	points.append(points[-1] + Vector(-iWidth, 0).rot(ori))
	points.append(points[0])
	trackObjects.append(draw(iName+"_track2", points))

	points = [pos]
	points.append(points[-1] + Vector(0, iLength/2+inGap).rot(ori))
	points.append(points[-1] + Vector(inGap+iWidth+iSize/2, 0).rot(ori))
	if(outGap-inGap!=0):
		points.append(points[-1] + Vector(0, outGap-inGap).rot(ori))
	points.append(points[-1] + Vector(outGap+iWidth+iSize/2, 0).rot(ori))
	points.append(points[-1] + Vector(0, -iLength-2*outGap).rot(ori))
	points.append(points[-1] + Vector(-outGap-iWidth-iSize/2, 0).rot(ori))
	if(outGap-inGap!=0):
		points.append(points[-1] + Vector(0, outGap-inGap).rot(ori))
	points.append(points[-1] + Vector(-inGap-iWidth-iSize/2, 0).rot(ori))
	points.append(points[0])
	gapObjects.append(draw(iName+"_gap", points))

	return [retIn, retOut]

def drawConnector(iName, iIn, iOut, iBoundLength, iSlope=1, iLineTest=False):

	pos, ori = Vector(iIn[POS]), Vector(iIn[ORI])
	_, _, inTrack, inGap = iIn
	_, _, outTrack, outGap = iOut
	adaptDist = abs(outTrack/2-inTrack/2)/iSlope
	retIn = [pos, -ori, inTrack, inGap]
	retOut = [pos+(adaptDist+inGap+iBoundLength)*ori, ori, outTrack, outGap]

	points = [pos + Vector(inGap, inTrack/2).rot(ori)]
	points.append(points[-1] + Vector(iBoundLength+iLineTest*inGap, 0).rot(ori))
	points.append(points[-1] + Vector(adaptDist, outTrack/2-inTrack/2).rot(ori))
	points.append(points[-1] + Vector(0, -outTrack).rot(ori))
	points.append(points[-1] + Vector(-adaptDist, outTrack/2-inTrack/2).rot(ori))
	points.append(points[-1] + Vector(-iBoundLength-iLineTest*inGap, 0).rot(ori))
	points.append(points[0])
	trackObjects.append(draw(iName+"_track1", points))

	points = [pos + Vector(0, inGap+0.5*inTrack).rot(ori)]
	points.append(points[-1] + Vector(inGap+iBoundLength, 0).rot(ori))
	points.append(points[-1] + Vector(adaptDist, (outGap-inGap)+(outTrack-inTrack)*0.5).rot(ori))
	points.append(points[-1] + Vector(0, -2*outGap-outTrack).rot(ori))
	points.append(points[-1] + Vector(-adaptDist, (outGap-inGap)+(outTrack-inTrack)*0.5).rot(ori))
	points.append(points[-1] + Vector(-(inGap+iBoundLength), 0).rot(ori))
	points.append(points[0])
	gapObjects.append(draw(iName+"_gap", points))
	
	if(iLineTest==False):
		points = [pos + Vector(0, inGap+0.5*inTrack).rot(ori)]
		points.append(points[-1] + Vector(inGap/2, 0).rot(ori))
		points.append(points[-1] + Vector(0, -2*inGap-inTrack).rot(ori))
		points.append(points[-1] + Vector(-inGap/2, 0).rot(ori))
		points.append(points[0])
		trackObjects.append(draw(iName+"_track2", points))

	return [retIn, retOut]

def drawAdaptor(iName, iIn, iOut, iSlope=1):

	pos, ori = Vector(iIn[POS]), Vector(iIn[ORI])
	_, _, inTrack, inGap = iIn
	_, _, outTrack, outGap = iOut
	retIn = [pos, -ori, inTrack, inGap]
	retOut = [pos+abs(outTrack/2-inTrack/2)*ori, ori, outTrack, outGap]
	adaptDist = abs(outTrack/2-inTrack/2)/iSlope

	points = [pos + Vector(0, inTrack/2).rot(ori)]
	points.append(points[-1] + Vector(adaptDist, outTrack/2-inTrack/2).rot(ori))
	points.append(points[-1] + Vector(0, -outTrack).rot(ori))
	points.append(points[-1] + Vector(-adaptDist, outTrack/2-inTrack/2).rot(ori))
	points.append(points[0])
	trackObjects.append(draw(iName+"_track", points))

	points = [pos + Vector(0, inGap+0.5*inTrack).rot(ori)]
	points.append(points[-1] + Vector(adaptDist, (outGap-inGap)+(outTrack-inTrack)*0.5).rot(ori))
	points.append(points[-1] + Vector(0, -2*outGap-outTrack).rot(ori))
	points.append(points[-1] + Vector(-adaptDist, (outGap-inGap)+(outTrack-inTrack)*0.5).rot(ori))
	points.append(points[0])
	gapObjects.append(draw(iName+"_gap", points))

	return [retIn, retOut]

def drawTriJunction(iName, iIn, iOut1, iOut3):

	pos, ori = Vector(iIn[POS]), Vector(iIn[ORI])
	_, _, inTrack, inGap = iIn
	_, _, out1Track, out1Gap = iOut1
	_, _, out3Track, out3Gap = iOut3
	retIn = [pos, -ori, inTrack, inGap]
	retOut1 = [pos+Vector(inGap+inTrack/2, inGap+inTrack/2).rot(ori), Vector(0, 1).rot(ori), out1Track, out1Gap]
	retOut3 = [pos+Vector(inGap+inTrack/2, -(inGap+inTrack/2)).rot(ori), Vector(0, -1).rot(ori), out3Track, out3Gap]
	
	points = [pos + Vector(0, inGap+inTrack/2).rot(ori)]
	points.append(points[-1] + Vector(2*inGap+inTrack, 0).rot(ori))
	points.append(points[-1] + Vector(0, -(2*inGap+inTrack)).rot(ori))
	points.append(points[-1] + Vector(-(2*inGap+inTrack), 0).rot(ori))
	points.append(points[0])
	gapObjects.append(draw(iName+"_gap", points))
	
	points = [pos + Vector(0, inTrack/2).rot(ori)]
	points.append(points[-1] + Vector(inGap, 0).rot(ori))
	points.append(points[-1] + Vector(0, inGap).rot(ori))
	points.append(points[-1] + Vector(inTrack, 0).rot(ori))
	points.append(points[-1] + Vector(0, -2*inGap-inTrack).rot(ori))
	points.append(points[-1] + Vector(-inTrack, 0).rot(ori))
	points.append(points[-1] + Vector(0, inGap).rot(ori))
	points.append(points[-1] + Vector(-inGap, 0).rot(ori))
	points.append(points[0])
	trackObjects.append(draw(iName+"_track", points))
	
	return [retIn, retOut3, retOut1]
	
def drawQuadJunction(iName, iIn, iOut1, iOut2, iOut3):

	pos, ori = Vector(iIn[POS]), Vector(iIn[ORI])
	_, _, inTrack, inGap = iIn
	_, _, out1Track, out1Gap = iOut1
	_, _, out2Track, out2Gap = iOut2
	_, _, out3Track, out3Gap = iOut3
	retIn = [pos, -ori, inTrack, inGap]
	retOut1 = [pos+Vector(inGap+inTrack/2, inGap+inTrack/2).rot(ori), Vector(0, 1).rot(ori), out1Track, out1Gap]
	retOut2 = [pos+Vector(2*inGap+inTrack, 0).rot(ori), Vector(1, 0).rot(ori), out2Track, out2Gap]
	retOut3 = [pos+Vector(inGap+inTrack/2, -(inGap+inTrack/2)).rot(ori), Vector(0, -1).rot(ori), out3Track, out3Gap]
	
	points = [pos + Vector(0, inGap+inTrack/2).rot(ori)]
	points.append(points[-1] + Vector(2*inGap+inTrack, 0).rot(ori))
	points.append(points[-1] + Vector(0, -(2*inGap+inTrack)).rot(ori))
	points.append(points[-1] + Vector(-(2*inGap+inTrack), 0).rot(ori))
	points.append(points[0])
	gapObjects.append(draw(iName+"_gap", points))
	
	points = [pos + Vector(0, inTrack/2).rot(ori)]
	points.append(points[-1] + Vector(inGap, 0).rot(ori))
	points.append(points[-1] + Vector(0, inGap).rot(ori))
	points.append(points[-1] + Vector(inTrack, 0).rot(ori))
	points.append(points[-1] + Vector(0, -inGap).rot(ori))
	points.append(points[-1] + Vector(inGap, 0).rot(ori))
	points.append(points[-1] + Vector(0, -inTrack).rot(ori))
	points.append(points[-1] + Vector(-inGap, 0).rot(ori))
	points.append(points[-1] + Vector(0, -inGap).rot(ori))
	points.append(points[-1] + Vector(-inTrack, 0).rot(ori))
	points.append(points[-1] + Vector(0, inGap).rot(ori))
	points.append(points[-1] + Vector(-inGap, 0).rot(ori))
	points.append(points[0])
	trackObjects.append(draw(iName+"_track", points))
	
	return [retIn, retOut3, retOut2, retOut1]
	
def drawEnd(iName, iIn):

	pos, ori = Vector(iIn[POS]), Vector(iIn[ORI])
	_, _, inTrack, inGap = iIn
	retIn = [pos, -ori, inTrack, inGap]
	
	points = [pos + Vector(0, inGap+inTrack/2).rot(ori)]
	points.append(points[-1] + Vector(2*inGap+inTrack, 0).rot(ori))
	points.append(points[-1] + Vector(0, -(2*inGap+inTrack)).rot(ori))
	points.append(points[-1] + Vector(-(2*inGap+inTrack), 0).rot(ori))
	points.append(points[0])
	gapObjects.append(draw(iName+"_gap", points))
	
	points = [pos + Vector(0, inTrack/2).rot(ori)]
	points.append(points[-1] + Vector(inGap+inTrack, 0).rot(ori))
	points.append(points[-1] + Vector(0, -inTrack).rot(ori))
	points.append(points[-1] + Vector(-inGap-inTrack, 0).rot(ori))
	points.append(points[0])
	trackObjects.append(draw(iName+"_track", points))
	
	return retIn

def drawCable(iName, iIn, iOut, iMaxFilet=0.5):

	inPos, inOri = Vector(iIn[POS]), Vector(iIn[ORI])
	outPos, outOri = Vector(iOut[POS]), Vector(iOut[ORI])
	_, _, track, gap = iIn

	if(iIn[TRACK] != iOut[TRACK] or iIn[GAP] != iOut[GAP]):
		if(iIn[TRACK]+iIn[GAP] > iOut[TRACK] + iOut[GAP]):
			_, newOut = drawAdaptor(iName+"_adaptor", iOut, [None, None, iIn[TRACK], iIn[GAP]])
			outPos = newOut[POS]
			outOri = newOut[ORI]
		else:
			_, newIn = drawAdaptor(iName+"_adaptor", iIn, [None, None, iOut[TRACK], iOut[GAP]])
			inPos = newIn[POS]
			inOri = newIn[ORI]
			track = newIn[TRACK]
			gap = newIn[GAP]

	if(inOri*outOri != 0):
		middle = 0.5*(inPos+outPos)

		if(inOri.y==0):
			middleTop = [middle, [0,1], track, gap]
			middleBottom = [middle, [0,-1], track, gap]
			if(inPos.y>outPos.y):
				drawCable(iName+"_in", [inPos, inOri, track, gap], middleTop, iMaxFilet)
				drawCable(iName+"_out", middleBottom, [outPos, outOri, track, gap], iMaxFilet)
				return
			else:
				drawCable(iName+"_in", [inPos, inOri, track, gap], middleBottom, iMaxFilet)
				drawCable(iName+"_out", middleTop, [outPos, outOri, track, gap], iMaxFilet)
				return
		else:
			middleRight = [middle, [1,0], track, gap]
			middleLeft = [middle, [-1,0], track, gap]
			if(inPos.x>outPos.x):
				drawCable(iName+"_in", [inPos, inOri, track, gap], middleRight, iMaxFilet)
				drawCable(iName+"_out", middleLeft, [outPos, outOri, track, gap], iMaxFilet)
				return
			else:
				drawCable(iName+"_in", [inPos, inOri, track, gap], middleLeft, iMaxFilet)
				drawCable(iName+"_out", middleRight, [outPos, outOri, track, gap], iMaxFilet)
				return
	else:
		inLength = (outPos-inPos).rot(inOri).abs().x
		outLength = (outPos-inPos).rot(inOri).abs().y

		points = [inPos -outOri*0.5*track]
		points.append(points[-1] + (inLength-0.5*track)*inOri)
		points.append(points[-1] + -(outLength-0.5*track)*outOri)
		points.append(points[-1] + track*inOri)
		points.append(points[-1] + (outLength+0.5*track)*outOri)
		points.append(points[-1] + -(inLength+0.5*track)*inOri)
		points.append(points[0])
		trackObjects.append(draw(iName+"_track", points))

		points = [inPos + -(0.5*track+gap)*outOri]
		points.append(points[-1] + (inLength-(0.5*track+gap))*inOri)
		points.append(points[-1] + -(outLength-(0.5*track+gap))*outOri)
		points.append(points[-1] + (track+2*gap)*inOri)
		points.append(points[-1] + (outLength+(0.5*track+gap))*outOri)
		points.append(points[-1] + -(inLength+(0.5*track+gap))*inOri)
		points.append(points[0])
		gapObjects.append(draw(iName+"_gap", points))
		
		filetRay = 0.5*min(iMaxFilet, min(inLength, outLength)-0.5*track-gap)
		if(iMaxFilet != None and filetRay >0):
			filet(iName+"_track", filetRay+gap, 1)
			filet(iName+"_track", filetRay+gap+track, 5)
			filet(iName+"_gap", filetRay, 1)
			filet(iName+"_gap", filetRay+2*gap+track, 5)

def drawWave(iName, iIn, iOut, iNbWaves, iRightShift, iLeftShift, iGndSize):

	pos, ori = Vector(iIn[POS]), Vector(iIn[ORI])
	_, _, track, gap = iIn
	period = 2*gap+track+iGndSize/2
	border = 2*gap+track+iGndSize

	retOut = [pos + (2*border+iNbWaves*period)*ori, ori, track, gap]
	retIn = [pos, -ori, track, gap]

	for i in range(iNbWaves+2):
		j = ((i%2)-0.5)*2
		waveIn = [pos + Vector(0, j*(iRightShift*(j%2)+iLeftShift*(1-j%2))).rot(ori) + (border+(i-1)*period)*ori, ori, track, gap]
		waveOut = [pos + Vector(0, -j*(iLeftShift*(j%2)+iRightShift*(1-j%2))).rot(ori) + (border+i*period)*ori, -ori, track, gap]

		if(i==0):
			waveIn=iIn
		if(i==iNbWaves+1):
			waveOut=[pos + (2*border+iNbWaves*period)*ori, -ori, track, gap]

		drawCable(iName+"_waveN"+str(i), waveIn, waveOut)

	return [retIn, retOut]


def drawJSJunc(iName, iIn, iOut, iSize, iWidth, iLength, iInduct=0.1):

	pos, ori = Vector(iIn[POS]), Vector(iIn[ORI])
	_, _, track, gap = iIn
	adaptDist = abs(iWidth/2-track/2)
	retIn = [pos, -ori, track, gap]
	retOut = [pos+abs(2*adaptDist+2*iLength+iSize)*ori, ori, track, gap]	
	
	points = [pos + Vector(0, 0.5*track).rot(ori)]
	points.append(points[-1] + Vector(adaptDist, iWidth/2-track/2).rot(ori))
	points.append(points[-1] + Vector(iLength, 0).rot(ori))
	points.append(points[-1] + Vector(0, -iWidth).rot(ori))
	points.append(points[-1] + Vector(-iLength, 0).rot(ori))
	points.append(points[-1] + Vector(-adaptDist, iWidth/2-track/2).rot(ori))
	points.append(points[0])
	trackObjects.append(draw(iName+"_track1", points))
	
	points = [pos + Vector(2*adaptDist+2*iLength+iSize, 0.5*track).rot(ori)]
	points.append(points[-1] + Vector(-adaptDist, iWidth/2-track/2).rot(ori))
	points.append(points[-1] + Vector(-iLength, 0).rot(ori))
	points.append(points[-1] + Vector(0, -iWidth).rot(ori))
	points.append(points[-1] + Vector(iLength, 0).rot(ori))
	points.append(points[-1] + Vector(adaptDist, iWidth/2-track/2).rot(ori))
	points.append(points[0])
	trackObjects.append(draw(iName+"_track2", points))

	points = [pos + Vector(0, 0.5*track+gap).rot(ori)]
	points.append(points[-1] + Vector(2*adaptDist+2*iLength+iSize, 0).rot(ori))
	points.append(points[-1] + Vector(0, -2*gap-track).rot(ori))
	points.append(points[-1] + Vector(-2*adaptDist-2*iLength-iSize, 0).rot(ori))
	points.append(points[0])
	gapObjects.append(draw(iName+"_gap", points))
	
	points = [pos + Vector(adaptDist+iLength, +0.5*iWidth).rot(ori)]
	points.append(points[-1] + Vector(iSize, 0).rot(ori))
	points.append(points[-1] + Vector(0, -iWidth).rot(ori))
	points.append(points[-1] + Vector(-iSize, 0).rot(ori))
	points.append(points[0])
	draw(iName+"_junc", points)
	assigneInduc(iName+"_junc", pos + Vector(adaptDist+iLength, 0).rot(ori), pos + Vector(adaptDist+iLength+iSize, 0).rot(ori), iInduct)
	
	return [retIn, retOut]

def drawSquide(iName, iIn, iOut, iSize, iWidth, iLength, iSpace, iInduct=0.1):

	pos, ori = Vector(iIn[POS]), Vector(iIn[ORI])
	_, _, track, gap = iIn
	adaptDist = abs((2*iWidth+iSpace)/2-track/2)
	retIn = [pos, -ori, track, gap]
	retOut = [pos+abs(2*adaptDist+2*iLength+iSize)*ori, ori, track, gap]
	retFluxDown = [pos+Vector(adaptDist+iLength+iSize/2, -gap-track/2).rot(ori), Vector(0, 1).rot(ori), None, None]
	retFluxUp = [pos+Vector(adaptDist+iLength+iSize/2, gap+track/2).rot(ori), Vector(0, -1).rot(ori), None, None]
	
	points = [pos + Vector(0, 0.5*track).rot(ori)]
	points.append(points[-1] + Vector(adaptDist, (2*iWidth+iSpace)/2-track/2).rot(ori))
	points.append(points[-1] + Vector(iLength, 0).rot(ori))
	points.append(points[-1] + Vector(0, -iWidth).rot(ori))
	points.append(points[-1] + Vector(-iLength, 0).rot(ori))
	points.append(points[-1] + Vector(0, -iSpace).rot(ori))
	points.append(points[-1] + Vector(iLength, 0).rot(ori))
	points.append(points[-1] + Vector(0, -iWidth).rot(ori))
	points.append(points[-1] + Vector(-iLength, 0).rot(ori))
	points.append(points[-1] + Vector(-adaptDist, (2*iWidth+iSpace)/2-track/2).rot(ori))
	points.append(points[0])
	trackObjects.append(draw(iName+"_track1", points))
	
	points = [pos + Vector(2*adaptDist+2*iLength+iSize, 0.5*track).rot(ori)]
	points.append(points[-1] + Vector(-adaptDist, (2*iWidth+iSpace)/2-track/2).rot(ori))
	points.append(points[-1] + Vector(-iLength, 0).rot(ori))
	points.append(points[-1] + Vector(0, -iWidth).rot(ori))
	points.append(points[-1] + Vector(iLength, 0).rot(ori))
	points.append(points[-1] + Vector(0, -iSpace).rot(ori))
	points.append(points[-1] + Vector(-iLength, 0).rot(ori))
	points.append(points[-1] + Vector(0, -iWidth).rot(ori))
	points.append(points[-1] + Vector(iLength, 0).rot(ori))
	points.append(points[-1] + Vector(adaptDist, (2*iWidth+iSpace)/2-track/2).rot(ori))
	points.append(points[0])
	trackObjects.append(draw(iName+"_track2", points))

	points = [pos + Vector(0, 0.5*track+gap).rot(ori)]
	points.append(points[-1] + Vector(2*adaptDist+2*iLength+iSize, 0).rot(ori))
	points.append(points[-1] + Vector(0, -2*gap-track).rot(ori))
	points.append(points[-1] + Vector(-2*adaptDist-2*iLength-iSize, 0).rot(ori))
	points.append(points[0])
	gapObjects.append(draw(iName+"_gap", points))
	
	points = [pos + Vector(adaptDist+iLength, iWidth+0.5*iSpace).rot(ori)]
	points.append(points[-1] + Vector(iSize, 0).rot(ori))
	points.append(points[-1] + Vector(0, -iWidth).rot(ori))
	points.append(points[-1] + Vector(-iSize, 0).rot(ori))
	points.append(points[0])
	draw(iName+"_junc1", points)
	#assigneInduc(iName+"_junc1", pos + Vector(adaptDist+iLength, 0.5*iWidth+0.5*iSpace).rot(ori), pos + Vector(adaptDist+iLength+iSize, 0.5*iWidth+0.5*iSpace).rot(ori), iInduct)
	
	points = [pos + Vector(adaptDist+iLength, -0.5*iSpace).rot(ori)]
	points.append(points[-1] + Vector(iSize, 0).rot(ori))
	points.append(points[-1] + Vector(0, -iWidth).rot(ori))
	points.append(points[-1] + Vector(-iSize, 0).rot(ori))
	points.append(points[0])
	draw(iName+"_junc2", points)
	#assigneInduc(iName+"_junc2", pos + Vector(adaptDist+iLength, -0.5*iWidth-0.5*iSpace).rot(ori), pos + Vector(adaptDist+iLength+iSize, -0.5*iWidth-0.5*iSpace).rot(ori), iInduct)
	
	return [retIn, retOut, retFluxUp, retFluxDown]

def drawFlux(iName, iIn, iSize, iMargin, iGndGap=None):

	pos, ori = Vector(iIn[POS]), Vector(iIn[ORI])
	_, _, track, gap = iIn
	retIn = [pos, -ori, track, gap]
	
	points = [pos + Vector(0, 0.5*track).rot(ori)]
	points.append(points[-1] + Vector(iMargin, 0).rot(ori))
	points.append(points[-1] + Vector(0, gap).rot(ori))
	points.append(points[-1] + Vector(iSize+2*track, 0).rot(ori))
	points.append(points[-1] + Vector(0, -2*track-2*gap).rot(ori))
	points.append(points[-1] + Vector(-2*track-iSize-iMargin, 0).rot(ori))
	points.append(points[-1] + Vector(0, track).rot(ori))
	points.append(points[-1] + Vector(iMargin+track+iSize, 0).rot(ori))
	points.append(points[-1] + Vector(0, 2*gap).rot(ori))
	points.append(points[-1] + Vector(-iSize, 0).rot(ori))
	points.append(points[-1] + Vector(0, -gap).rot(ori))
	points.append(points[-1] + Vector(-iMargin-track, 0).rot(ori))
	points.append(points[0])
	trackObjects.append(draw(iName+"_track1", points))
	
	return retIn

#Dessin du chip et de ses connecteurs

chipLength, chipWidth = 8.12, 8.67
pcbTrack, pcbGap, boundLength, boundSlope = 0.3, 0.2, 0.2, 0.5
track, gap = 0.084, 0.05

con1, con2, con3, con4, con5 = 4.06, 3.01, 7.537, 3.01, 6.36

chipIn, chipOut = [[0, con1], [1, 0], pcbTrack, pcbGap], [None, None, track, gap]
tomoIn, tomoOut = [[con2, chipLength], [0, -1], pcbTrack, pcbGap], [None, None, track, gap]
qubitIn, qubitOut = [[con3, chipLength], [0, -1], pcbTrack, pcbGap], [None, None, track, gap]
fluxIn, fluxOut = [[con4, 0], [0, 1], pcbTrack, pcbGap], [None, None, track, gap]
memoryIn, memoryOut = [[con5, 0], [0, 1], pcbTrack, pcbGap], [None, None, track, gap]

chipIn, chipOut = drawConnector("chip_con", chipIn, chipOut, boundLength, boundSlope)
tomoIn, tomoOut = drawConnector("tomo_con", tomoIn, tomoOut, boundLength, boundSlope)
qubitIn, qubitOut = drawConnector("qubit_con", qubitIn, qubitOut, boundLength, boundSlope)
fluxIn, fluxOut = drawConnector("flux_con", fluxIn, fluxOut, boundLength, boundSlope)
memoryIn, memoryOut = drawConnector("memory_con", memoryIn, memoryOut, boundLength, boundSlope)

#Dessin de la croix centrale

crossGap = 0.4
crossSize = LitExp("$cross_size", 0.54)

cross1In = [[memoryIn[POS].x-crossGap*gap-track/2, chipIn[POS].y], [1, 0], track, crossGap*gap]
cross1OutRight, cross1OutLeft = [None, None, track, crossGap*gap], [None, None, track, crossGap*gap]
cross1OutUp, cross1OutRight, cross1OutLeft = drawTriJunction("cross1", cross1In, cross1OutRight, cross1OutLeft)

crossJSJuncIn, crossJSJuncOut = [cross1OutRight[POS], [0, 1], track, crossGap*gap], [None, None, track, crossGap*gap]
crossJSJuncIn, crossJSJuncOut = drawJSJunc("memory_jsjunc", crossJSJuncIn, crossJSJuncOut, 0.02, 0.01, 0.02, 8.2)

cross2In = [crossJSJuncOut[POS]+Vector(crossGap*gap+track/2, crossGap*gap+track/2), [-1, 0], track, crossGap*gap]
cross2OutLeft, cross2OutRight = [None, None, track, crossGap*gap], [None, None, track, crossGap*gap]
cross2OutDown, cross2OutLeft, cross2OutRight = drawTriJunction("cross2", cross2In, cross2OutLeft, cross2OutRight)

#Dessin de la piste d'entree du chip

chipCapaPos, chipWaveLength = LitExp("$chip_capa_pos", 0.6), LitExp("$chip_wave_length", 0.7)

chipCapa1In, chipCapa1Out = [chipOut[POS]+Vector(chipCapaPos, 0), [1, 0], track, gap], [None, None, track, gap]
chipCapa1In, chipCapa1Out = drawCapa("chip_capa1", chipCapa1In, chipCapa1Out, 0.3, track, 0.05)

chipCapa2In, chipCapa2Out = [cross1OutUp[POS]+Vector(-crossSize, 0), [-1, 0], track, crossGap*gap], [None, None, track, gap]
chipCapa2In, chipCapa2Out = drawCapa("chip_capa2", chipCapa2In, chipCapa2Out, 0.3, track, 0.05)

chipSquideIn, chipSquideOut = [chipCapa1Out[POS]*0.5+chipCapa2Out[POS]*0.5+Vector(-0.052, 0), [1, 0], track, gap], [None, None, track, gap]
chipSquideIn, chipSquideOut, fluxFluxIn, _ = drawSquide("chip_squide", chipSquideIn, chipSquideOut, 0.02, 0.01, 0.02, 0.02, 2.62)

chipWave1In, chipWave1Out = [chipCapa1Out[POS], [1, 0], track, gap], [None, None, track, gap]
chipWave1In, chipWave1Out = drawWave("chip1_wave", chipWave1In, chipWave1Out, 1, chipWaveLength, chipWaveLength, 0.4)

chipWave2In, chipWave2Out = [chipCapa2Out[POS], [-1, 0], track, gap], [None, None, track, gap]
chipWave2In, chipWave2Out = drawWave("chip2_wave", chipWave2In, chipWave2Out, 1, chipWaveLength, chipWaveLength, 0.4)

drawCable("chip_cable1", chipOut, chipCapa1In)
drawCable("chip_cable2", chipWave1Out, chipSquideIn)
drawCable("chip_cable3", chipSquideOut, chipWave2Out)
drawCable("chip_cable4", chipCapa2In, cross1OutUp)

#Dessin la piste de lecture du qubit

qubitCapaIn, qubitCapaOut = [cross2OutDown[POS]+Vector(crossSize, 0), [1, 0], track, crossGap*gap], [None, None, track, gap]
qubitCapaIn, qubitCapaOut = drawCapa("qubit_capa", qubitCapaIn, qubitCapaOut, 0.11, track, 0.31)

qubitDoubleJuncOut = [qubitCapaOut[POS]+Vector(0.3, 0.3), [0, 1], track, gap]
qubitDoubleJuncIn = [qubitCapaOut[POS]+Vector(0.3, 0.3), [0, -1], track, gap]

drawCable("qubit_cable1", qubitDoubleJuncOut, qubitOut)
drawCable("qubit_cable2", qubitCapaOut, qubitDoubleJuncIn)
drawCable("qubit_cable3", qubitCapaIn, cross2OutDown)

#Dessin de la piste de la tomographie

tomoDecal, tomoCapaPos = LitExp("$tomo_decal", 2.0), LitExp("$tomo_capa_pos", 0.2)

tomoCapa1In, tomoCapa1Out = [tomoOut[POS]+Vector(0, -tomoCapaPos), [0, -1], track, gap], [None, None, track, gap]
tomoCapa1In, tomoCapa1Out = drawCapa("tomo_capa1", tomoCapa1In, tomoCapa1Out, 0.19, track/2, 0.05)

tomoCapa2In, tomoCapa2Out = [cross2OutRight[POS]+Vector(0, crossSize), [0, 1], track, crossGap*gap], [None, None, track, gap]
tomoCapa2In, tomoCapa2Out = drawCapa("tomo_capa2", tomoCapa2In, tomoCapa2Out, 0.3, track, 0.05)

tomoDoubleJunc1In = [[tomoCapa1Out[POS].x, tomoCapa1Out[POS].y*0.70+tomoCapa2Out[POS].y*0.3]+Vector(-tomoDecal, 0), [0, 1], track, gap]
tomoDoubleJunc1Out = [[tomoCapa1Out[POS].x, tomoCapa1Out[POS].y*0.70+tomoCapa2Out[POS].y*0.3]+Vector(-tomoDecal, 0), [0, -1], track, gap]

tomoDoubleJunc2In = [[tomoCapa1Out[POS].x, tomoCapa1Out[POS].y*0.5+tomoCapa2Out[POS].y*0.5], [-1, 0], track, gap]
tomoDoubleJunc2Out = [[tomoCapa1Out[POS].x, tomoCapa1Out[POS].y*0.5+tomoCapa2Out[POS].y*0.5], [1, 0], track, gap]

drawCable("tomo_cable1", tomoOut, tomoCapa1In)
drawCable("tomo_cable2", tomoCapa1Out, tomoDoubleJunc1In)
drawCable("tomo_cable3", tomoDoubleJunc1Out, tomoDoubleJunc2In)
drawCable("tomo_cable4", tomoDoubleJunc2Out, tomoCapa2Out)
drawCable("tomo_cable5", tomoCapa2In, cross2OutRight)

#Dessin de la piste de la memoire

memoryCapa1Pos, memoryWaveLength = LitExp("$memory_capa_pos", 0.2), LitExp("$memory_wave_length", 1.2)

memoryCapa1In, memoryCapa1Out = [memoryOut[POS]+Vector(0, memoryCapa1Pos), [0, 1], track, gap], [None, None, track, gap]
memoryCapa1In, memoryCapa1Out = drawCapa("memory_capa1", memoryCapa1In, memoryCapa1Out, 0.12, track/2, 0.2)

memoryCapa2In, memoryCapa2Out = [cross1OutLeft[POS]+Vector(0, -crossSize), [0, -1], track, gap*crossGap], [None, None, track, gap]
memoryCapa2In, memoryCapa2Out = drawCapa("memory_capa2", memoryCapa2In, memoryCapa2Out, 0.16, track/2, 0.05)

memoryWaveIn, memoryWaveOut = [memoryCapa1Out[POS], [0, 1], track, gap], [None, None, track, gap]
memoryWaveIn, memoryWaveOut = drawWave("memory_wave", memoryWaveIn, memoryWaveOut, 2, memoryWaveLength, memoryWaveLength, 0.35)

drawCable("memory_cable1", memoryOut, memoryCapa1In)
drawCable("memory_cable2", memoryWaveOut, memoryCapa2Out)
drawCable("memory_cable3", memoryCapa2In, cross1OutLeft)

#Dessin de la ligne d'application du flux

fluxFluxIn[TRACK], fluxFluxIn[GAP] = 0.002, 0.012
fluxFluxIn = drawFlux("flux_flux", fluxFluxIn, 0.02, 0.01)

drawCable("flux_cable", fluxOut, fluxFluxIn)

#Finalisation globale du dessin

trackObject = fuse(trackObjects)
gapObject = fuse(gapObjects)

substract(gapObject, "Ground_plane")
assignPerfE(trackObject, "track")