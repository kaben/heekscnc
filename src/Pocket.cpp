// Pocket.cpp
/*
 * Copyright (c) 2009, Dan Heeks
 * This program is released under the BSD license. See the file COPYING for
 * details.
 */

#include "stdafx.h"
#include "Pocket.h"
#include "CNCConfig.h"
#include "ProgramCanvas.h"
#include "Program.h"
#include "interface/HeeksObj.h"
#include "interface/PropertyDouble.h"
#include "interface/PropertyLength.h"
#include "interface/PropertyString.h"
#include "interface/PropertyChoice.h"
#include "interface/PropertyVertex.h"
#include "interface/PropertyCheck.h"
#include "tinyxml/tinyxml.h"
#include "CTool.h"
#include "CNCPoint.h"
#include "Reselect.h"
#include "MachineState.h"
#include "PocketDlg.h"

/* LibAREA headers. */
#include "Area.h"
#include "Curve.h"

// /* OpenVoronoi headers. */
#include "openvoronoi/voronoidiagram.hpp"
#include "openvoronoi/polygon_interior_filter.hpp"
#include "openvoronoi/medial_axis_filter.hpp"
#include "openvoronoi/offset.hpp"

#include "interface/TestMacros.h"

#include <boost/progress.hpp>

#include <sstream>
#include <cmath>

extern CHeeksCADInterface* heeksCAD;

// static
double CPocket::max_deviation_for_spline_to_arc = 0.1;

CPocketParams::CPocketParams()
{
	m_step_over = 0.0;
	m_material_allowance = 0.0;
	m_starting_place = true;
	m_keep_tool_down_if_poss = true;
	m_use_zig_zag = true;
	m_zig_angle = 0.0;
	m_zig_unidirectional = false;
	m_entry_move = ePlunge;
}

void CPocketParams::set_initial_values(const CTool::ToolNumber_t tool_number)
{
    if (tool_number > 0)
    {
        CTool *pTool = CTool::Find(tool_number);
        if (pTool != NULL)
        {
            m_step_over = pTool->CuttingRadius() * 3.0 / 5.0;
        }
    }
}

static void on_set_entry_move(int value, HeeksObj* object)
{
	((CPocket*)object)->m_pocket_params.m_entry_move = (CPocketParams::eEntryStyle) value;
	((CPocket*)object)->WriteDefaultValues();
}

static void on_set_step_over(double value, HeeksObj* object)
{
	((CPocket*)object)->m_pocket_params.m_step_over = value;
	((CPocket*)object)->WriteDefaultValues();
}

static void on_set_material_allowance(double value, HeeksObj* object)
{
	((CPocket*)object)->m_pocket_params.m_material_allowance = value;
	((CPocket*)object)->WriteDefaultValues();
}

static void on_set_starting_place(int value, HeeksObj* object)
{
	((CPocket*)object)->m_pocket_params.m_starting_place = value;
	((CPocket*)object)->WriteDefaultValues();
}

static void on_set_keep_tool_down(bool value, HeeksObj* object)
{
	((CPocket*)object)->m_pocket_params.m_keep_tool_down_if_poss = value;
	((CPocket*)object)->WriteDefaultValues();
}

static void on_set_use_zig_zag(bool value, HeeksObj* object)
{
	((CPocket*)object)->m_pocket_params.m_use_zig_zag = value;
	((CPocket*)object)->WriteDefaultValues();
}

static void on_set_zig_angle(double value, HeeksObj* object)
{
	((CPocket*)object)->m_pocket_params.m_zig_angle = value;
	((CPocket*)object)->WriteDefaultValues();
}

static void on_set_zig_uni(bool value, HeeksObj* object)
{
	((CPocket*)object)->m_pocket_params.m_zig_unidirectional = value;
	((CPocket*)object)->WriteDefaultValues();
}

void CPocketParams::GetProperties(CPocket* parent, std::list<Property *> *list)
{
	list->push_back(new PropertyLength(_("step over"), m_step_over, parent, on_set_step_over));
	list->push_back(new PropertyLength(_("material allowance"), m_material_allowance, parent, on_set_material_allowance));
	{
		std::list< wxString > choices;
		choices.push_back(_("Boundary"));
		choices.push_back(_("Center"));
		list->push_back(new PropertyChoice(_("starting_place"), choices, m_starting_place, parent, on_set_starting_place));
	}
	{
		std::list< wxString > choices;
		choices.push_back(_("Plunge"));
		choices.push_back(_("Ramp"));
		choices.push_back(_("Helical"));
		list->push_back(new PropertyChoice(_("entry_move"), choices, m_entry_move, parent, on_set_entry_move));
	}
	list->push_back(new PropertyCheck(_("keep tool down"), m_keep_tool_down_if_poss, parent, on_set_keep_tool_down));
	list->push_back(new PropertyCheck(_("use zig zag"), m_use_zig_zag, parent, on_set_use_zig_zag));
	if(m_use_zig_zag)
	{
		list->push_back(new PropertyDouble(_("zig angle"), m_zig_angle, parent, on_set_zig_angle));
		list->push_back(new PropertyCheck(_("unidirectional"), m_zig_unidirectional, parent, on_set_zig_uni));
	}
}

void CPocketParams::WriteXMLAttributes(TiXmlNode *root)
{
	TiXmlElement * element;
	element = heeksCAD->NewXMLElement( "params" );
	heeksCAD->LinkXMLEndChild( root,  element );
	element->SetDoubleAttribute( "step", m_step_over);
	element->SetDoubleAttribute( "mat", m_material_allowance);
	element->SetAttribute( "from_center", m_starting_place);
	element->SetAttribute( "keep_tool_down", m_keep_tool_down_if_poss ? 1:0);
	element->SetAttribute( "use_zig_zag", m_use_zig_zag ? 1:0);
	element->SetDoubleAttribute( "zig_angle", m_zig_angle);
	element->SetAttribute( "zig_unidirectional", m_zig_unidirectional ? 1:0);
	element->SetAttribute( "entry_move", (int) m_entry_move);
}

void CPocketParams::ReadFromXMLElement(TiXmlElement* pElem)
{
	pElem->Attribute("step", &m_step_over);
	pElem->Attribute("mat", &m_material_allowance);
	pElem->Attribute("from_center", &m_starting_place);
	int int_for_bool = false;
	pElem->Attribute("keep_tool_down", &int_for_bool);
	m_keep_tool_down_if_poss = (int_for_bool != 0);
	pElem->Attribute("use_zig_zag", &int_for_bool);
	m_use_zig_zag = (int_for_bool != 0);
	pElem->Attribute("zig_angle", &m_zig_angle);
	pElem->Attribute("zig_unidirectional", &int_for_bool);
	m_zig_unidirectional = (int_for_bool != 0);
	int int_for_entry_move = (int) ePlunge;
	pElem->Attribute("entry_move", &int_for_entry_move);
	m_entry_move = (eEntryStyle) int_for_entry_move;
}

//static wxString WriteCircleDefn(HeeksObj* sketch, CMachineState *pMachineState) {
//#ifdef UNICODE
//	std::wostringstream gcode;
//#else
//	std::ostringstream gcode;
//#endif
//	gcode.imbue(std::locale("C"));
//	gcode << std::setprecision(10);
//	std::list<std::pair<int, gp_Pnt> > points;
//	span_object->GetCentrePoint(c);
//
//	// Setup the four arcs that will make up the circle using UNadjusted
//	// coordinates first so that the offsets align with the X and Y axes.
//	double small_amount = 0.001;
//	double radius = heeksCAD->CircleGetRadius(span_object);
//
//	points.push_back(std::make_pair(LINEAR, gp_Pnt(c[0], c[1] + radius, c[2]))); // north
//	points.push_back(std::make_pair(CW, gp_Pnt(c[0] + radius, c[1], c[2]))); // east
//	points.push_back(std::make_pair(CW, gp_Pnt(c[0], c[1] - radius, c[2]))); // south
//	points.push_back(std::make_pair(CW, gp_Pnt(c[0] - radius, c[1], c[2]))); // west
//	points.push_back(std::make_pair(CW, gp_Pnt(c[0], c[1] + radius, c[2]))); // north
//
//	CNCPoint centre(pMachineState->Fixture().Adjustment(c));
//
//	gcode << _T("c = area.Curve()\n");
//	for (std::list<std::pair<int, gp_Pnt> >::iterator l_itPoint =
//			points.begin(); l_itPoint != points.end(); l_itPoint++) {
//		CNCPoint pnt = pMachineState->Fixture().Adjustment(l_itPoint->second);
//
//		gcode << _T("c.append(area.Vertex(") << l_itPoint->first
//				<< _T(", area.Point(");
//		gcode << pnt.X(true) << (_T(", ")) << pnt.Y(true);
//		gcode << _T("), area.Point(") << centre.X(true) << _T(", ")
//				<< centre.Y(true) << _T(")))\n");
//	} // End for
//	gcode << _T("a.append(c)\n");
//}
static wxString WriteSketchDefn(HeeksObj* sketch, CMachineState *pMachineState, int &num_curves)
{
#ifdef UNICODE
	std::wostringstream gcode;
#else
    std::ostringstream gcode;
#endif
    gcode.imbue(std::locale("C"));
	gcode << std::setprecision(10);

	bool started = false;

	double prev_e[3];

	std::list<HeeksObj*> new_spans;
	for(HeeksObj* span = sketch->GetFirstChild(); span; span = sketch->GetNextChild())
	{
		if(span->GetType() == SplineType)
		{
			heeksCAD->SplineToBiarcs(span, new_spans, CPocket::max_deviation_for_spline_to_arc);
		}
		else
		{
			new_spans.push_back(span->MakeACopy());
		}
	}

	for(std::list<HeeksObj*>::iterator It = new_spans.begin(); It != new_spans.end(); It++)
	{
		HeeksObj* span_object = *It;

		double s[3] = {0, 0, 0};
		double e[3] = {0, 0, 0};
		double c[3] = {0, 0, 0};

		if(span_object){
			int type = span_object->GetType();

			if(type == LineType || type == ArcType)
			{
				span_object->GetStartPoint(s);
#ifdef STABLE_OPS_ONLY
				CNCPoint start(s);
#else
				CNCPoint start(pMachineState->Fixture().Adjustment(s));
#endif

				if(started && (fabs(s[0] - prev_e[0]) > 0.0001 || fabs(s[1] - prev_e[1]) > 0.0001))
				{
					gcode << _T("a.append(c)\n");
					started = false;
          num_curves++;
				}

				if(!started)
				{
					gcode << _T("c = area.Curve()\n");
					gcode << _T("c.append(area.Vertex(0, area.Point(") << start.X(true) << _T(", ") << start.Y(true) << _T("), area.Point(0, 0)))\n");
					started = true;
				}
				span_object->GetEndPoint(e);
#ifdef STABLE_OPS_ONLY
				CNCPoint end(e);
#else
				CNCPoint end(pMachineState->Fixture().Adjustment(e));
#endif

				if(type == LineType)
				{
					gcode << _T("c.append(area.Vertex(0, area.Point(") << end.X(true) << _T(", ") << end.Y(true) << _T("), area.Point(0, 0)))\n");
				}
				else if(type == ArcType)
				{
					span_object->GetCentrePoint(c);
#ifdef STABLE_OPS_ONLY
					CNCPoint centre(c);
#else
					CNCPoint centre(pMachineState->Fixture().Adjustment(c));
#endif

					double pos[3];
					heeksCAD->GetArcAxis(span_object, pos);
					int span_type = (pos[2] >=0) ? 1:-1;
					gcode << _T("c.append(area.Vertex(") << span_type << _T(", area.Point(") << end.X(true) << _T(", ") << end.Y(true);
					gcode << _T("), area.Point(") << centre.X(true) << _T(", ") << centre.Y(true) << _T(")))\n");
				}
				memcpy(prev_e, e, 3*sizeof(double));
			} // End if - then
			else
			{
				if (type == CircleType)
				{
					if(started)
					{
						gcode << _T("a.append(c)\n");
						started = false;
            num_curves++;
					}

					std::list< std::pair<int, gp_Pnt > > points;
					span_object->GetCentrePoint(c);

					// Setup the four arcs that will make up the circle using UNadjusted
					// coordinates first so that the offsets align with the X and Y axes.
					double radius = heeksCAD->CircleGetRadius(span_object);

					points.push_back( std::make_pair(0, gp_Pnt( c[0], c[1] + radius, c[2] )) ); // north
					points.push_back( std::make_pair(-1, gp_Pnt( c[0] + radius, c[1], c[2] )) ); // east
					points.push_back( std::make_pair(-1, gp_Pnt( c[0], c[1] - radius, c[2] )) ); // south
					points.push_back( std::make_pair(-1, gp_Pnt( c[0] - radius, c[1], c[2] )) ); // west
					points.push_back( std::make_pair(-1, gp_Pnt( c[0], c[1] + radius, c[2] )) ); // north

#ifdef STABLE_OPS_ONLY
					CNCPoint centre(c);
#else
					CNCPoint centre(pMachineState->Fixture().Adjustment(c));
#endif

					gcode << _T("c = area.Curve()\n");
					for (std::list< std::pair<int, gp_Pnt > >::iterator l_itPoint = points.begin(); l_itPoint != points.end(); l_itPoint++)
					{
#ifdef STABLE_OPS_ONLY
						CNCPoint pnt( l_itPoint->second );
#else
						CNCPoint pnt = pMachineState->Fixture().Adjustment( l_itPoint->second );
#endif

						gcode << _T("c.append(area.Vertex(") << l_itPoint->first << _T(", area.Point(");
						gcode << pnt.X(true) << (_T(", ")) << pnt.Y(true);
						gcode << _T("), area.Point(") << centre.X(true) << _T(", ") << centre.Y(true) << _T(")))\n");
					} // End for
					gcode << _T("a.append(c)\n");
          num_curves++;
				}
			} // End if - else
		}
	}

	if(started)
	{
		gcode << _T("a.append(c)\n");
		started = false;
    num_curves++;
	}

	// delete the spans made
	for(std::list<HeeksObj*>::iterator It = new_spans.begin(); It != new_spans.end(); It++)
	{
		HeeksObj* span = *It;
		delete span;
	}

	gcode << _T("\n");
	return(wxString(gcode.str().c_str()));
}

const wxBitmap &CPocket::GetIcon()
{
	if(!m_active)return GetInactiveIcon();
	static wxBitmap* icon = NULL;
	if(icon == NULL)icon = new wxBitmap(wxImage(theApp.GetResFolder() + _T("/icons/pocket.png")));
	return *icon;
}

Python CPocket::AppendTextToProgram(CMachineState *pMachineState)
{
  printf("entered ...\n");
	Python python;
  int num_curves = 0;

#ifdef OP_SKETCHES_AS_CHILDREN
	ReloadPointers();   // Make sure all the m_sketches values have been converted into children.
#endif

	CTool *pTool = CTool::Find( m_tool_number );
	if (pTool == NULL)
	{
		wxMessageBox(_T("Cannot generate GCode for pocket without a tool assigned"));
		return(python);
	} // End if - then


	python << CDepthOp::AppendTextToProgram(pMachineState);

	python << _T("a = area.Area()\n");
	python << _T("entry_moves = []\n");

#ifdef OP_SKETCHES_AS_CHILDREN
	int num_children = GetNumChildren();
	int child_num = 0;
	dprintf("iterating through %d children to generate pockets ...\n", num_children);
    for (HeeksObj *object = GetFirstChild(); object != NULL; object = GetNextChild())
    {
#else
	for (std::list<int>::iterator It = m_sketches.begin(); It != m_sketches.end(); It++)
    {
		HeeksObj* object = heeksCAD->GetIDObject(SketchType, *It);
#endif
        child_num++;
	    dprintf("(child_num %d/%d) considering child ...\n", child_num, num_children);
		if (object->GetType() != SketchType)
		{
	        dprintf("(child_num %d/%d) skipping non-sketch child ...\n", child_num, num_children);
			continue;	// Skip private fixture objects.
		}

		if(object == NULL) {
			wxMessageBox(wxString::Format(_("Pocket operation - Sketch doesn't exist")));
			continue;
		}
		int type = object->GetType();
		double c[3] = {0, 0, 0};
		double radius;

		switch (type) {

		case CircleType:
	        dprintf("(child_num %d/%d) circle! ...\n", child_num, num_children);
			if (m_pocket_params.m_entry_move == CPocketParams::eHelical) {
				GetCentrePoint(c);
				radius = heeksCAD->CircleGetRadius(object);
				python << _T("# entry_moves.append(circle(") << c[0]/theApp.m_program->m_units << _T(", ") << c[1]/theApp.m_program->m_units << _T(", ")<< c[2]/theApp.m_program->m_units << _T(", ") << radius/theApp.m_program->m_units ;
				python << _T("))\n") ;
			} else {
				wxLogMessage(_T("circle found in pocket operation (id=%d) but entry move is not helical, id=%d"), GetID(),object->GetID());
			}
			continue;

		case PointType:
	        dprintf("(child_num %d/%d) point! ...\n", child_num, num_children);
			if (m_pocket_params.m_entry_move == CPocketParams::eHelical) {
				memset( c, 0, sizeof(c) );
				heeksCAD->VertexGetPoint( object, c);
				python << _T("# entry_moves.append(point(") << c[0]/theApp.m_program->m_units << _T(", ") << c[1]/theApp.m_program->m_units << _T("))\n");

			} else {
				wxLogMessage(_T("point found in pocket operation (id=%d) but entry move is not helical, id=%d"), GetID(), object->GetID());
			}
			continue;

		default:
			break;
		}
		if (object->GetNumChildren() == 0){
			wxMessageBox(wxString::Format(_("Pocket operation - Sketch %d has no children"), object->GetID()));
			continue;
		}

	    dprintf("(child_num %d/%d) GetSketchOrder(...) ...\n", child_num, num_children);
		HeeksObj* re_ordered_sketch = NULL;
		SketchOrderType order = heeksCAD->GetSketchOrder(object);
	    dprintf("(child_num %d/%d) considering whether to reorder ...\n", child_num, num_children);
		if( 	(order != SketchOrderTypeCloseCW) &&
			(order != SketchOrderTypeCloseCCW) &&
			(order != SketchOrderTypeMultipleCurves) &&
			(order != SketchOrderHasCircles))
		{
	        dprintf("(child_num %d/%d) reordering ...\n", child_num, num_children);
			re_ordered_sketch = object->MakeACopy();
	        dprintf("(child_num %d/%d) ReOrderSketch(...) ...\n", child_num, num_children);
			heeksCAD->ReOrderSketch(re_ordered_sketch, SketchOrderTypeReOrder);
			object = re_ordered_sketch;
	        dprintf("(child_num %d/%d) GetSketchOrder(...) (again) ...\n", child_num, num_children);
			order = heeksCAD->GetSketchOrder(object);
			if(	(order != SketchOrderTypeCloseCW) &&
				(order != SketchOrderTypeCloseCCW) &&
				(order != SketchOrderTypeMultipleCurves) &&
				(order != SketchOrderHasCircles))
			{
				switch(heeksCAD->GetSketchOrder(object))
				{
				case SketchOrderTypeOpen:
					{
						wxMessageBox(wxString::Format(_("Pocket operation - Sketch must be a closed shape - sketch %d"), object->m_id));
						delete re_ordered_sketch;
						continue;
					}
					break;

				default:
					{
						wxMessageBox(wxString::Format(_("Pocket operation - Badly ordered sketch - sketch %d"), object->m_id));
						delete re_ordered_sketch;
						continue;
					}
					break;
				}
			}
		}

		if(object)
		{
	        dprintf("(child_num %d/%d) WriteSketchDefn(...) ...\n", child_num, num_children);
			python << WriteSketchDefn(object, pMachineState, num_curves);
		} else {
	        dprintf("(child_num %d/%d) order is null , so not calling WriteSketchDefn(...) ...\n", child_num, num_children);
        }

		if(re_ordered_sketch)
		{
			delete re_ordered_sketch;
		}
	    dprintf("(child_num %d/%d) ... done considering this child.\n", child_num, num_children);
	} // End for

	// Pocket the area
  if (0 < num_curves)
  {
	  // reorder the area, the outside curves must be made anti-clockwise and the insides clockwise
	  python << _T("a.Reorder()\n");

	  // start - assume we are at a suitable clearance height

	  // make a parameter of area_funcs.pocket() eventually
	  // 0..plunge, 1..ramp, 2..helical
	  python << _T("entry_style = ") <<  m_pocket_params.m_entry_move << _T("\n");

		python << _T("area_funcs.pocket(a, tool_diameter/2, ");
		python << m_pocket_params.m_material_allowance / theApp.m_program->m_units;
		python << _T(", rapid_safety_space, start_depth, final_depth, ");
		python << m_pocket_params.m_step_over / theApp.m_program->m_units;
		python << _T(", step_down, clearance, ");
		python << m_pocket_params.m_starting_place;
		python << (m_pocket_params.m_keep_tool_down_if_poss ? _T(", True") : _T(", False"));
		python << (m_pocket_params.m_use_zig_zag ? _T(", True") : _T(", False"));
		python << _T(", ") << m_pocket_params.m_zig_angle;
		python << _T(",") << (m_pocket_params.m_zig_unidirectional ? _T("True") : _T("False"));
		python << _T(")\n");
  }
  else
  {
		python << _T("\n");
		python << _T("comment('In the python code that generated this gcode,')\n");
		python << _T("comment('an area was created that lacks sufficient curves')\n");
		python << _T("comment('to successfully call the pocketing function,')\n");
		python << _T("comment('wherefore I refuse cowardly to even try')\n");
		python << _T("comment('because I think I might crash if I do.')\n");
		python << _T("\n");
  }

	// rapid back up to clearance plane
	python << _T("rapid(z = clearance)\n");

  printf("... done.\n");
	return(python);

} // End AppendTextToProgram() method


void CPocket::WriteDefaultValues()
{
	CDepthOp::WriteDefaultValues();

	CNCConfig config(CPocketParams::ConfigScope());
	config.Write(_T("StepOver"), m_pocket_params.m_step_over);
	config.Write(_T("MaterialAllowance"), m_pocket_params.m_material_allowance);
	config.Write(_T("FromCenter"), m_pocket_params.m_starting_place);
	config.Write(_T("KeepToolDown"), m_pocket_params.m_keep_tool_down_if_poss);
	config.Write(_T("UseZigZag"), m_pocket_params.m_use_zig_zag);
	config.Write(_T("ZigAngle"), m_pocket_params.m_zig_angle);
	config.Write(_T("ZigUnidirectional"), m_pocket_params.m_zig_unidirectional);
	config.Write(_T("DecentStrategy"), m_pocket_params.m_entry_move);
}

void CPocket::ReadDefaultValues()
{
	CDepthOp::ReadDefaultValues();

	CNCConfig config(CPocketParams::ConfigScope());
	config.Read(_T("StepOver"), &m_pocket_params.m_step_over, 1.0);
	config.Read(_T("MaterialAllowance"), &m_pocket_params.m_material_allowance, 0.2);
	config.Read(_T("FromCenter"), &m_pocket_params.m_starting_place, 1);
	config.Read(_T("KeepToolDown"), &m_pocket_params.m_keep_tool_down_if_poss, true);
	config.Read(_T("UseZigZag"), &m_pocket_params.m_use_zig_zag, false);
	config.Read(_T("ZigAngle"), &m_pocket_params.m_zig_angle);
	config.Read(_T("ZigUnidirectional"), &m_pocket_params.m_zig_unidirectional, false);
	int int_for_entry_move = CPocketParams::ePlunge;
	config.Read(_T("DecentStrategy"), &int_for_entry_move);
	m_pocket_params.m_entry_move = (CPocketParams::eEntryStyle) int_for_entry_move;
}

void CPocket::glCommands(bool select, bool marked, bool no_color)
{
	CDepthOp::glCommands( select, marked, no_color );
}

void CPocket::GetProperties(std::list<Property *> *list)
{
#ifdef OP_SKETCHES_AS_CHILDREN
	AddSketchesProperties(list, this);
#else
	AddSketchesProperties(list, m_sketches);
#endif
	m_pocket_params.GetProperties(this, list);
	CDepthOp::GetProperties(list);
}

HeeksObj *CPocket::MakeACopy(void)const
{
	return new CPocket(*this);
}

void CPocket::CopyFrom(const HeeksObj* object)
{
	operator=(*((CPocket*)object));
}

CPocket::CPocket( const CPocket & rhs ) : CDepthOp(rhs)
{
	m_sketches.clear();
	std::copy( rhs.m_sketches.begin(), rhs.m_sketches.end(), std::inserter( m_sketches, m_sketches.begin() ) );
	m_pocket_params = rhs.m_pocket_params;
}

CPocket & CPocket::operator= ( const CPocket & rhs )
{
	if (this != &rhs)
	{
		CDepthOp::operator=(rhs);
		m_sketches.clear();
		std::copy( rhs.m_sketches.begin(), rhs.m_sketches.end(), std::inserter( m_sketches, m_sketches.begin() ) );

		m_pocket_params = rhs.m_pocket_params;
		// static double max_deviation_for_spline_to_arc;
	}

	return(*this);
}

bool CPocket::CanAddTo(HeeksObj* owner)
{
	return ((owner != NULL) && (owner->GetType() == OperationsType));
}

void CPocket::WriteXML(TiXmlNode *root)
{
	TiXmlElement * element = heeksCAD->NewXMLElement( "Pocket" );
	heeksCAD->LinkXMLEndChild( root,  element );
	m_pocket_params.WriteXMLAttributes(element);

	// write sketch ids
	for(std::list<int>::iterator It = m_sketches.begin(); It != m_sketches.end(); It++)
	{
		int sketch = *It;
		TiXmlElement * sketch_element = heeksCAD->NewXMLElement( "sketch" );
		heeksCAD->LinkXMLEndChild( element, sketch_element );
		sketch_element->SetAttribute("id", sketch);
	}

	WriteBaseXML(element);
}

// static member function
HeeksObj* CPocket::ReadFromXMLElement(TiXmlElement* element)
{
	CPocket* new_object = new CPocket;

	std::list<TiXmlElement *> elements_to_remove;

	// read profile parameters
	TiXmlElement* params = heeksCAD->FirstNamedXMLChildElement(element, "params");
	if(params)
	{
		new_object->m_pocket_params.ReadFromXMLElement(params);
		elements_to_remove.push_back(params);
	}

	// read sketch ids
	for(TiXmlElement* sketch = heeksCAD->FirstNamedXMLChildElement(element, "sketch"); sketch; sketch = sketch->NextSiblingElement())
	{
		if ((wxString(Ctt(sketch->Value())) == wxString(_T("sketch"))) &&
			(sketch->Attribute("id") != NULL) &&
			(sketch->Attribute("title") == NULL))
		{
			int id = 0;
			sketch->Attribute("id", &id);
			if(id)
			{
				new_object->m_sketches.push_back(id);
			}

			elements_to_remove.push_back(sketch);
		} // End if - then
	}

	for (std::list<TiXmlElement*>::iterator itElem = elements_to_remove.begin(); itElem != elements_to_remove.end(); itElem++)
	{
		heeksCAD->RemoveXMLChild( element, *itElem);
	}

	// read common parameters
	new_object->ReadBaseXML(element);

	return new_object;
}

CPocket::CPocket(const std::list<int> &sketches, const int tool_number )
	: CDepthOp(GetTypeString(), &sketches, tool_number ), m_sketches(sketches)
{
	ReadDefaultValues();
	m_pocket_params.set_initial_values(tool_number);

#ifdef OP_SKETCHES_AS_CHILDREN
	for (Sketches_t::iterator sketch = m_sketches.begin(); sketch != m_sketches.end(); sketch++)
	{
		HeeksObj *object = heeksCAD->GetIDObject( SketchType, *sketch );
		if (object != NULL)
		{
			Add( object, NULL );
		}
	}

	m_sketches.clear();
#endif
}

CPocket::CPocket(const std::list<HeeksObj *> &sketches, const int tool_number )
	: CDepthOp(GetTypeString(), sketches, tool_number )
{
	ReadDefaultValues();
	m_pocket_params.set_initial_values(tool_number);

#ifdef OP_SKETCHES_AS_CHILDREN
	for (std::list<HeeksObj *>::const_iterator sketch = sketches.begin(); sketch != sketches.end(); sketch++)
	{
		Add( *sketch, NULL );
	}
#endif
}



/**
	The old version of the CDrilling object stored references to graphics as type/id pairs
	that get read into the m_symbols list.  The new version stores these graphics references
	as child elements (based on ObjList).  If we read in an old-format file then the m_symbols
	list will have data in it for which we don't have children.  This routine converts
	these type/id pairs into the HeeksObj pointers as children.
 */
#ifdef OP_SKETCHES_AS_CHILDREN
void CPocket::ReloadPointers()
{
	for (Sketches_t::iterator symbol = m_sketches.begin(); symbol != m_sketches.end(); symbol++)
	{
		HeeksObj *object = heeksCAD->GetIDObject( SketchType, *symbol );
		if (object != NULL)
		{
			Add( object, NULL );
		}
	}

	m_sketches.clear();	// We don't want to convert them twice.

	CDepthOp::ReloadPointers();
}
#endif



/**
	This method adjusts any parameters that don't make sense.  It should report a list
	of changes in the list of strings.
 */
std::list<wxString> CPocket::DesignRulesAdjustment(const bool apply_changes)
{
	std::list<wxString> changes;

	int num_sketches = 0;
#ifdef OP_SKETCHES_AS_CHILDREN
    for (HeeksObj *object = GetFirstChild(); object != NULL; object = GetNextChild())
    {
		if (object->GetType() == SketchType)
		{
		    num_sketches++;
		}
#else
	for (std::list<int>::iterator It = m_sketches.begin(); It != m_sketches.end(); It++)
    {
		HeeksObj* object = heeksCAD->GetIDObject(SketchType, *It);
#endif
	} // End if - then

	if (num_sketches == 0)
	{
#ifdef UNICODE
			std::wostringstream l_ossChange;
#else
			std::ostringstream l_ossChange;
#endif

			l_ossChange << _("No valid sketches upon which to act for pocket operations") << " id='" << m_id << "'\n";
			changes.push_back(l_ossChange.str().c_str());
	} // End if - then


	if (m_tool_number > 0)
	{
		// Make sure the hole depth isn't greater than the tool's depth.
		CTool *pCutter = (CTool *) CTool::Find( m_tool_number );

		if ((pCutter != NULL) && (pCutter->m_params.m_cutting_edge_height < m_depth_op_params.m_final_depth))
		{
			// The tool we've chosen can't cut as deep as we've setup to go.

#ifdef UNICODE
			std::wostringstream l_ossChange;
#else
			std::ostringstream l_ossChange;
#endif

			l_ossChange << _("Adjusting depth of pocket") << " id='" << m_id << "' " << _("from") << " '"
				<< m_depth_op_params.m_final_depth << "' " << _("to") << " "
				<< pCutter->m_params.m_cutting_edge_height << " " << _("due to cutting edge length of selected tool") << "\n";
			changes.push_back(l_ossChange.str().c_str());

			if (apply_changes)
			{
				m_depth_op_params.m_final_depth = pCutter->m_params.m_cutting_edge_height;
			} // End if - then
		} // End if - then

		// Also make sure the 'step-over' distance isn't larger than the tool's diameter.
		if ((pCutter != NULL) && ((pCutter->CuttingRadius(false) * 2.0) < m_pocket_params.m_step_over))
		{
			wxString change;
			change << _("The step-over distance for pocket (id=");
			change << m_id;
			change << _(") is larger than the tool's diameter");
			changes.push_back(change);

			if (apply_changes)
			{
				m_pocket_params.m_step_over = (pCutter->CuttingRadius(false) * 2.0);
			} // End if - then
		} // End if - then
	} // End if - then

	std::list<wxString> depth_op_changes = CDepthOp::DesignRulesAdjustment( apply_changes );
	std::copy( depth_op_changes.begin(), depth_op_changes.end(), std::inserter( changes, changes.end() ) );

	return(changes);

} // End DesignRulesAdjustment() method

static void on_set_spline_deviation(double value, HeeksObj* object){
	CPocket::max_deviation_for_spline_to_arc = value;
	CPocket::WriteToConfig();
}

// static
void CPocket::GetOptions(std::list<Property *> *list)
{
	list->push_back ( new PropertyDouble ( _("Pocket spline deviation"), max_deviation_for_spline_to_arc, NULL, on_set_spline_deviation ) );
}

// static
void CPocket::ReadFromConfig()
{
	CNCConfig config(CPocketParams::ConfigScope());
	config.Read(_T("PocketSplineDeviation"), &max_deviation_for_spline_to_arc, 0.1);
}

// static
void CPocket::WriteToConfig()
{
	CNCConfig config(CPocketParams::ConfigScope());
	config.Write(_T("PocketSplineDeviation"), max_deviation_for_spline_to_arc);
}

static ReselectSketches reselect_sketches;

void CPocket::GetTools(std::list<Tool*>* t_list, const wxPoint* p)
{
	reselect_sketches.m_sketches = &m_sketches;
	reselect_sketches.m_object = this;
	t_list->push_back(&reselect_sketches);

    CDepthOp::GetTools( t_list, p );
}

bool CPocketParams::operator==(const CPocketParams & rhs) const
{
	if (m_starting_place != rhs.m_starting_place) return(false);
	if (m_material_allowance != rhs.m_material_allowance) return(false);
	if (m_step_over != rhs.m_step_over) return(false);
	if (m_keep_tool_down_if_poss != rhs.m_keep_tool_down_if_poss) return(false);
	if (m_use_zig_zag != rhs.m_use_zig_zag) return(false);
	if (m_zig_angle != rhs.m_zig_angle) return(false);
	if (m_zig_unidirectional != rhs.m_zig_unidirectional) return(false);
	if (m_entry_move != rhs.m_entry_move) return(false);

	return(true);
}

bool CPocket::operator==(const CPocket & rhs) const
{
	if (m_pocket_params != rhs.m_pocket_params) return(false);

	return(CDepthOp::operator==(rhs));
}

static bool OnEdit(HeeksObj* object)
{
	PocketDlg dlg(heeksCAD->GetMainFrame(), (CPocket*)object);
	if(dlg.ShowModal() == wxID_OK)
	{
		dlg.GetData((CPocket*)object);
		((CPocket*)object)->WriteDefaultValues();
		return true;
	}
	return false;
}

void CPocket::GetOnEdit(bool(**callback)(HeeksObj*))
{
	*callback = OnEdit;
}

bool CPocket::Add(HeeksObj* object, HeeksObj* prev_object)
{
	return CDepthOp::Add(object, prev_object);
}


int CPocket::ConvertSketchToArea(
  HeeksObj *object,
  CArea &area,
  CMachineState *pMachineState
){
  int num_curves = 0;

  // Find and skip things we can't process...
  if (object->GetType() != SketchType) { return num_curves; } // Skip private fixture objects.
  if (object == NULL) { return num_curves; } // Skip null objects.
  if (object->GetNumChildren() == 0){ return num_curves; } // Skip empty objects.

  // Check for badly-ordered sketches, and try to fix.
  HeeksObj* re_ordered_sketch = NULL;
  SketchOrderType order = heeksCAD->GetSketchOrder(object);
  if(SketchNeedsReordering(order)){
    re_ordered_sketch = object->MakeACopy();
    heeksCAD->ReOrderSketch(re_ordered_sketch, SketchOrderTypeReOrder);
    object = re_ordered_sketch;
    order = heeksCAD->GetSketchOrder(object);
    if(SketchNeedsReordering(order)){ // Skip badly-ordered sketches that can't be fixed.
      delete re_ordered_sketch;
      return num_curves;
    }
  }
  if(object == NULL) { return num_curves; } // Sanity check, in case reordered sketch is null.

  CCurve curve;
  bool started = false;
  double prev_e[3];

  // Extract sketch elements for analysis.
  std::list<HeeksObj*> new_spans;
  for(HeeksObj* span = object->GetFirstChild(); span; span = object->GetNextChild()) {
    if(span->GetType() == SplineType) {
        heeksCAD->SplineToBiarcs(span, new_spans, CPocket::max_deviation_for_spline_to_arc);
    } else { new_spans.push_back(span->MakeACopy()); }
  }
  // Analyze each sketch element, and load into area object.
  for(std::list<HeeksObj*>::iterator It = new_spans.begin(); It != new_spans.end(); It++) {
    HeeksObj* span_object = *It;
    double s[3] = {0, 0, 0};
    double e[3] = {0, 0, 0};
    double c[3] = {0, 0, 0};
    
    if(span_object){
      int type = span_object->GetType();

      if(type == LineType || type == ArcType) {
        // Handle arcs and lines.
        span_object->GetStartPoint(s);
        CNCPoint start(pMachineState->Fixture().Adjustment(s));
        if(started && (fabs(s[0] - prev_e[0]) > 0.0001 || fabs(s[1] - prev_e[1]) > 0.0001)) {
          // Complete previous loop, load into loops list, get ready for next loop.
          area.append(curve);
          curve.m_vertices.clear();
          started = false;
          num_curves++;
        }

        // Check whether to start a new curve.
        if(!started) {
          curve.append(CVertex(0, Point(start.X(true), start.Y(true)), Point()));
          started = true;
        }

        // Now handle the other end of the edge or arc.
        span_object->GetEndPoint(e);
        memcpy(prev_e, e, 3*sizeof(double));
        CNCPoint end(pMachineState->Fixture().Adjustment(e));
        if(type == LineType) {
          // Finish handling line.
          curve.append(CVertex(0, Point(end.X(true), end.Y(true)), Point()));
        } else if(type == ArcType) {
          // Finish handling arc.
          span_object->GetCentrePoint(c);
          CNCPoint centre(pMachineState->Fixture().Adjustment(c));
          double pos[3];
          heeksCAD->GetArcAxis(span_object, pos);
          int span_type = (pos[2] >=0) ? 1:-1;
          curve.append(CVertex(span_type, Point(end.X(true), end.Y(true)), Point(centre.X(true), centre.Y(true))));
        }
        memcpy(prev_e, e, 3*sizeof(double));
      } else {
        // Handle circles.
        if (type == CircleType) {
          if(started) {
            // Complete previous loop, load into loops list, get ready for next loop.
            area.append(curve);
            curve.m_vertices.clear();
            started = false;
          num_curves++;
            // FIXME: improperly-closed loop may cause problems later.
          }

          std::list< std::pair<int, gp_Pnt > > points;
          span_object->GetCentrePoint(c);

          // Setup the four arcs that will make up the circle using UNadjusted
          // coordinates first so that the offsets align with the X and Y axes.
          double radius = heeksCAD->CircleGetRadius(span_object);

          points.push_back( std::make_pair(0, gp_Pnt( c[0], c[1] + radius, c[2] )) ); // north
          points.push_back( std::make_pair(-1, gp_Pnt( c[0] + radius, c[1], c[2] )) ); // east
          points.push_back( std::make_pair(-1, gp_Pnt( c[0], c[1] - radius, c[2] )) ); // south
          points.push_back( std::make_pair(-1, gp_Pnt( c[0] - radius, c[1], c[2] )) ); // west
          points.push_back( std::make_pair(-1, gp_Pnt( c[0], c[1] + radius, c[2] )) ); // north
          CNCPoint centre(pMachineState->Fixture().Adjustment(c));
          curve.m_vertices.clear();
          for (std::list< std::pair<int, gp_Pnt > >::iterator l_itPoint = points.begin(); l_itPoint != points.end(); l_itPoint++) {
            CNCPoint pnt = pMachineState->Fixture().Adjustment( l_itPoint->second );
            curve.append(CVertex(l_itPoint->first, Point(pnt.X(true), pnt.Y(true)), Point(centre.X(true), centre.Y(true))));
          }
          // Complete loop, load into loops list, get ready for next loop.
          area.append(curve);
          curve.m_vertices.clear();
          num_curves++;
        }
      }
    }
  }

  if(started) {
    // Complete previous loop, load into loops list, get ready for next loop.
    area.append(curve);
    curve.m_vertices.clear();
    started = false;
    num_curves++;
    // FIXME: improperly-closed loop may cause problems later.
  }
  // delete the spans made
  for(std::list<HeeksObj*>::iterator It = new_spans.begin(); It != new_spans.end(); It++) {
    HeeksObj* span = *It;
    delete span;
  }
  if(re_ordered_sketch) { delete re_ordered_sketch; }

  return num_curves;
}

bool CPocket::ConvertSketchesToArea(
  std::list<HeeksObj *> &sketches,
  CArea &area,
  CMachineState *pMachineState
)
{
  int num_curves = 0;
  for (std::list<HeeksObj *>::iterator it = sketches.begin(); it != sketches.end(); it++) {
    HeeksObj* object = *it;
    num_curves += ConvertSketchToArea(object, area, pMachineState);
  }

  double did_generate_area = true;
  if (0 < num_curves) {
    // Reorder the area. The outside curves must be made anti-clockwise and the
    // insides clockwise.
    area.Reorder();
  } else {
    did_generate_area = false;
  }
  return did_generate_area;
}

//void _pre_intersects(Seg &s1, Seg &s2, ovd::Point &p, ovd::Point &r, ovd::Point &q, ovd::Point &s) {
//  p = s1.first;
//  r = s1.second - p;
//  q = s2.first;
//  s = s2.second - q;
//}
//
//void _pre_crosses(ovd::Point &p, ovd::Point &r, ovd::Point &q, ovd::Point &s, double &qmpxs, double &rxs) {
//  qmpxs = (q-p).cross(s);
//  rxs = r.cross(s);
//}
//
//bool _collinear(double &qmpxs, double &rxs) {
//  if ((rxs == 0 ) /* parallel */ && (qmpxs == 0) /* collinear */)
//    return true;
//  return false;
//}
//
//bool intersects(Seg& s1, Seg& s2, bool &parallel, Seg& o11, Seg& o12, Seg& o21, Seg& o22) {
//  ovd::Point p = s1.first;
//  ovd::Point r = s1.second - p;
//  ovd::Point q = s2.first;
//  ovd::Point s = s2.second - q;
//
//  double qmpxs = (q-p).cross(s);
//  double rxs = r.cross(s);
//
//  if (rsx == 0) {
//    parallel = true;
//    if (qmpxs == 0) { return true; }
//    return false;
//  } else { parallel = false; }
//}

bool intersects(Seg& s1, Seg& s2, bool& parallel,  double& t, double& u) {
  ovd::Point p = s1.first;
  ovd::Point r = s1.second - p;
  ovd::Point q = s2.first;
  ovd::Point s = s2.second - q;

  double qmpxs = (q-p).cross(s);
  double rxs = r.cross(s);

  if (rxs == 0) {
    parallel = true;
    if (qmpxs == 0) {
      return true;
    }
    return false;
  } else {
    parallel = false;
  }

  double qmpxr = (q-p).cross(r);
  double irxs = 1./rxs;
  t = qmpxs * irxs;
  u = qmpxr * irxs;
  if ( (0.0<=t) && (t<=1.0) && (0.0<=u) && (u<=1.0) ) { return true; }
  else { return false; }
}

//bool segment_intersects(Segs& segs, Seg& s, Seg &os) {
//  bool parallel;
//  double t, u;
//  BOOST_FOREACH(Seg& seg, segs) {
//    if (intersects(seg, s, parallel, t, u)) {
//      if (!joins(seg, s)) {
//        dprintf("intersection: s: (%g,%g)-(%g,%g) ...\n", s.first.x, s.first.y, s.second.x, s.second.y);
//        dprintf("intersection: seg: (%g,%g)-(%g,%g) ...\n", seg.first.x, seg.first.y, seg.second.x, seg.second.y);
//        os = seg;
//        return true;
//      }
//    }
//  }
//  return false; // no intersections found
//}
bool joins(Seg &s1, Seg &s2) {
  if (s1.first == s2.first) return true;
  if (s1.first == s2.second) return true;
  if (s1.second == s2.first) return true;
  if (s1.second == s2.second) return true;
  return false;
}

// test if s intersects with any of the segments in segs
// return true if there is an intersection, otherwise false
bool segment_intersects(Segs& segs, Seg& s, Seg &os) {
  bool parallel;
  double t, u;
  BOOST_FOREACH(Seg& seg, segs) {
    if (intersects(seg, s, parallel, t, u)) {
      if (!joins(seg, s)) {
        dprintf("intersection: s: (%g,%g)-(%g,%g) ...\n", s.first.x, s.first.y, s.second.x, s.second.y);
        dprintf("intersection: seg: (%g,%g)-(%g,%g) ...\n", seg.first.x, seg.first.y, seg.second.x, seg.second.y);
        os = seg;
        return true;
      }
    }
  }
  return false; // no intersections found
}


void TranslateScale::set_bits(int _bits) {
    bits = _bits;
    bits_power = pow((double)2., (double)bits);
    inv_bits_power = 1./bits_power;
}
void TranslateScale::set(ovd::Point &p) {
    min_x = max_x = p.x;
    min_y = max_y = p.y;
}
void TranslateScale::update(ovd::Point &p) {
    if (first) {
        set(p);
        first = false;
    } else {
        if (p.x < min_x) min_x = p.x;
        if (p.y < min_y) min_y = p.y;
        if (p.x > max_x) max_x = p.x;
        if (p.y > max_y) max_y = p.y;
    }
}
void TranslateScale::set_translate_scale(){
    d_x = max_x - min_x;
    d_y = max_y - min_y;
    c_x = min_x + d_x/2.;
    c_y = min_y + d_y/2.;
    s_x = 1./d_x;
    s_y = 1./d_y;
    is_x = d_x;
    is_y = d_y;
}
void TranslateScale::set_translate_scale_fixed_aspect(){
    set_translate_scale();
    is = d_x;
    if (is < d_y) is = d_y;
    s = 1./is;
    s_x = s;
    s_y = s;
    is_x = is;
    is_y = is;
}
void TranslateScale::translate(ovd::Point &p) {
    p.x -= c_x;
    p.y -= c_y;
}
void TranslateScale::scale(ovd::Point &p) {
    p.x *= s_x;
    p.y *= s_y;
}
void TranslateScale::scale(double &d) {
    d *= s;
}
void TranslateScale::translate_scale(ovd::Point &p) {
    translate(p);
    scale(p);
}
void TranslateScale::inv_translate(ovd::Point &p) {
    p.x += c_x;
    p.y += c_y;
}
void TranslateScale::inv_scale(ovd::Point &p) {
    p.x *= is_x;
    p.y *= is_y;
}
void TranslateScale::inv_scale(double &d) {
    d *= is;
}
void TranslateScale::inv_scale_translate(ovd::Point &p) {
    inv_scale(p);
    inv_translate(p);
}
void TranslateScale::round(double &d) {
    int exponent;
    double significand = frexp(d, &exponent);
    significand = int(significand*bits_power)*inv_bits_power;
    d = ldexp(significand, exponent);
}
void TranslateScale::round(ovd::Point &p) {
    if(0.<bits){
        round(p.x);
        round(p.y);
    }
}
bool CPocket::SketchNeedsReordering(SketchOrderType order) {
  return((order != SketchOrderTypeCloseCW) && (order != SketchOrderTypeCloseCCW) && (order != SketchOrderTypeMultipleCurves) && (order != SketchOrderHasCircles));
}

bool CPocket::ConvertSketchesToOVDOffsetLoops(
  std::list<HeeksObj *> &sketches,
  ovd::OffsetLoops &loops,
  CMachineState *pMachineState
)
{
  int num_curves = 0;
  PtIDs pt_ids;
  for (std::list<HeeksObj *>::iterator it = sketches.begin(); it != sketches.end(); it++) {
    HeeksObj* object = *it;

    // Find and skip things we can't process...
    if (object->GetType() != SketchType) { continue; } // Skip private fixture objects.
    if (object == NULL) { continue; } // Skip null objects.
    if (object->GetNumChildren() == 0){ continue; } // Skip empty objects.

    // Check for badly-ordered sketches, and try to fix.
    HeeksObj* re_ordered_sketch = NULL;
    SketchOrderType order = heeksCAD->GetSketchOrder(object);
    if(SketchNeedsReordering(order)){
      re_ordered_sketch = object->MakeACopy();
      heeksCAD->ReOrderSketch(re_ordered_sketch, SketchOrderTypeReOrder);
      object = re_ordered_sketch;
      order = heeksCAD->GetSketchOrder(object);
      if(SketchNeedsReordering(order)){ // Skip badly-ordered sketches that can't be fixed.
        delete re_ordered_sketch;
        continue;
      }
    }
    if(object == NULL) { continue; } // Sanity check, in case reordered sketch is null.

    ovd::OffsetLoop loop;
    bool started = false;
    double prev_e[3];

    // Extract sketch elements for analysis.
    std::list<HeeksObj*> new_spans;
    for(HeeksObj* span = object->GetFirstChild(); span; span = object->GetNextChild()) {
      if(span->GetType() == SplineType) {
          heeksCAD->SplineToBiarcs(span, new_spans, CPocket::max_deviation_for_spline_to_arc);
      } else { new_spans.push_back(span->MakeACopy()); }
    }
    // Analyze each sketch element.
    for(std::list<HeeksObj*>::iterator It = new_spans.begin(); It != new_spans.end(); It++) {
      HeeksObj* span_object = *It;
      double s[3] = {0, 0, 0};
      double e[3] = {0, 0, 0};
      double c[3] = {0, 0, 0};
      
      if(span_object){
        int type = span_object->GetType();
        
        if(type == LineType || type == ArcType) {
          // Handle arcs and lines.
          span_object->GetStartPoint(s);
          CNCPoint start(pMachineState->Fixture().Adjustment(s));
          if(started && (fabs(s[0] - prev_e[0]) > 0.0001 || fabs(s[1] - prev_e[1]) > 0.0001)) {
            // Complete previous loop, load into loops list, get ready for next loop.
            loops.push_back(loop);
            loop = ovd::OffsetLoop();
            started = false;
            num_curves++;
          }

          // Check whether to start a new loop.
          if(!started) {
            ovd::Point startpt(start.X(true), start.Y(true));
            ovd::OffsetVertex lpt(startpt);
            loop.push_back(lpt);
            started = true;
          }

          // Now handle the other end of the line or arc.
          span_object->GetEndPoint(e);
          memcpy(prev_e, e, 3*sizeof(double));
          CNCPoint end(pMachineState->Fixture().Adjustment(e));
          if(type == LineType) {
            // Finish handling line.
            loop.push_back(ovd::OffsetVertex(ovd::Point(end.X(true), end.Y(true))));
          } else if(type == ArcType) {
            // Finish handling arc.
            span_object->GetCentrePoint(c);
            CNCPoint centre(pMachineState->Fixture().Adjustment(c));
            double pos[3];
            heeksCAD->GetArcAxis(span_object, pos);
            bool cw = (pos[2] >= 0)?(false):(true);
            loop.push_back( ovd::OffsetVertex( ovd::Point( end.X(true), end.Y(true)), 0., ovd::Point( centre.X(true), centre.Y(true)), cw));
          }
          memcpy(prev_e, e, 3*sizeof(double));
        } else {
          // Handle circles.
          if (type == CircleType) {
            if(started) {
              // Complete previous loop, load into loops list, get ready for next loop.
              loops.push_back(loop);
              loop = ovd::OffsetLoop();
              started = false;
              num_curves++;
              // FIXME: improperly-closed loop may cause problems later.
            }

            std::list< std::pair<int, gp_Pnt > > points;
            span_object->GetCentrePoint(c);

            // Setup the four arcs that will make up the circle using UNadjusted
            // coordinates first so that the offsets align with the X and Y axes.
            double radius = heeksCAD->CircleGetRadius(span_object);

            points.push_back( std::make_pair(0, gp_Pnt( c[0], c[1] + radius, c[2] )) ); // north
            points.push_back( std::make_pair(-1, gp_Pnt( c[0] + radius, c[1], c[2] )) ); // east
            points.push_back( std::make_pair(-1, gp_Pnt( c[0], c[1] - radius, c[2] )) ); // south
            points.push_back( std::make_pair(-1, gp_Pnt( c[0] - radius, c[1], c[2] )) ); // west
            points.push_back( std::make_pair(-1, gp_Pnt( c[0], c[1] + radius, c[2] )) ); // north
            CNCPoint centre(pMachineState->Fixture().Adjustment(c));
            loop = ovd::OffsetLoop();
            for (std::list< std::pair<int, gp_Pnt > >::iterator l_itPoint = points.begin(); l_itPoint != points.end(); l_itPoint++) {
              CNCPoint pnt = pMachineState->Fixture().Adjustment( l_itPoint->second );
              bool cw = (l_itPoint->first == 1)?(false):(true);
              loop.push_back( ovd::OffsetVertex( ovd::Point( pnt.X(true), pnt.Y(true)), 0., ovd::Point( centre.X(true), centre.Y(true)), cw));
            }
            // Complete loop, load into loops list, get ready for next loop.
            loops.push_back(loop);
            loop = ovd::OffsetLoop();
            started = false;
            num_curves++;
          }
        }
      }
    }

    if(started) {
      // Complete previous loop, load into loops list, get ready for next loop.
      loops.push_back(loop);
      loop = ovd::OffsetLoop();
      started = false;
      num_curves++;
      // FIXME: improperly-closed loop may cause problems later.
    }
    // delete the spans made
    for(std::list<HeeksObj*>::iterator It = new_spans.begin(); It != new_spans.end(); It++) {
      HeeksObj* span = *It;
      delete span;
    }
    
    if(re_ordered_sketch) { delete re_ordered_sketch; }
  }
  double did_generate_loops = true;
  if (0 < num_curves) {
    did_generate_loops = false;
  }
  return did_generate_loops;
}

#define PI 3.1415926535897932
#define CIRCLE_FUZZ 1.e-9

void rotate(double &x, double &y, double c, double s) {
  double tx = x*c - y*s;
  y = x*s + y*c;
  x = tx;
}

void CPocket::OVDArcToLines(ovd::Point& prev_pt, ovd::OffsetVertex& v, ovd::OffsetLoop& loop, int steps_per_circle, double ds)
{
  if((v.r == -1.) || ((v.p - prev_pt).norm() <= 0.01)){
    loop.push_back(v);
  } else {
    ovd::Point start(prev_pt - v.c);
    ovd::Point end(v.p - v.c);
    double theta1 = std::atan2(start.x, start.y);
    double theta2 = std::atan2(end.x, end.y);
    if (!v.cw) { while ((theta2-theta1) > -CIRCLE_FUZZ) { theta2 -= 2*PI; } }
    else { while ((theta2-theta1) < CIRCLE_FUZZ) { theta2 += 2*PI; } }
    double dtheta = theta2-theta1;
    double arclength = v.r*dtheta;
    int steps;

    if (0 < steps_per_circle) {
      double dlength = steps_per_circle*arclength/(2*PI);
      steps = (arclength/dlength);
    } else if (0 < ds) {
      steps = (arclength/ds);
    } else {
      double dlength = arclength*0.1;
      steps = (arclength/dlength);
    }
    if (steps < 4) { steps = 4; }
    //dprintf("steps: %d\n", steps);
    double rsteps = 1./double(steps);
    double dc = cos(-dtheta*rsteps);
    double ds = sin(-dtheta*rsteps);

    double tx = start.x, ty = start.y;
    for (int i = 0; i < steps; i++) {
      rotate(tx, ty, dc, ds);
      loop.push_back(ovd::OffsetVertex(v.c + ovd::Point(tx, ty)));
    }
  }
}

ovd::OffsetLoop CPocket::OVDLoopArcsToLines(ovd::OffsetLoop& loop, int steps_per_circle, double ds)
{
  ovd::OffsetLoop out_loop;
  int num_vertices = loop.size();
  if (0 < num_vertices) {
    OVDArcToLines(loop[num_vertices-1].p, loop[0], out_loop, steps_per_circle, ds);
    for (int i=1; i < num_vertices; i++) { OVDArcToLines(loop[i-1].p, loop[i], out_loop); }
  }
  return out_loop;
}

bool CPocket::GetOVDOffsetLoopScaling(
  ovd::OffsetLoop &loop,
  TranslateScale &ts
){
  BOOST_FOREACH( ovd::OffsetVertex lpt, loop ) { ts.update(lpt.p); }
  return true;
}

bool CPocket::GetOVDOffsetLoopsScaling(
  ovd::OffsetLoops &loops,
  TranslateScale &ts
){
  // Find boundary containing all analyzed elements, and use this info to scale
  // and translate all elements into a region that OpenVoronoi can handle.
  dprintf("finding outer boundary containing all analyzed elements ...\n");
  BOOST_FOREACH( ovd::OffsetLoop loop, loops ) { GetOVDOffsetLoopScaling(loop, ts); }
  ts.set_translate_scale_fixed_aspect();
  return true;
}

bool CPocket::ScaleOVDOffsetLoops(
  ovd::OffsetLoops &loops,
  TranslateScale &ts
){
  for (unsigned int i=0; i<loops.size(); i++) {
    for (unsigned int j=0; j<loops[i].size(); j++) {
      ts.translate_scale(loops[i][j].p);
      if (loops[i][j].r != -1) {
        ts.translate_scale(loops[i][j].c);
        ts.scale(loops[i][j].r);
      }
    }
  }
  return true;
}

bool CPocket::InvScaleOVDOffsetLoops(
  ovd::OffsetLoops &loops,
  TranslateScale &ts
){
  for (unsigned int i=0; i<loops.size(); i++) {
    for (unsigned int j=0; j<loops[i].size(); j++) {
      ts.inv_scale_translate(loops[i][j].p);
      if (loops[i][j].r != -1) {
        ts.inv_scale_translate(loops[i][j].c);
        ts.inv_scale(loops[i][j].r);
      }
    }
  }
  return true;
}

bool CPocket::OVDFilterDuplicates(ovd::OffsetLoops &loops, Pts& pts, Segs& segs){
  ovd::Point last;
  ovd::Point next;
  Seg seg, interseg;

  bool first = true;
  int num_loops = loops.size();
  int loop_num = 0;
  BOOST_FOREACH( ovd::OffsetLoop lp, loops ) { // loop through each loop
    loop_num++;
    first = true;
    dprintf("adding points for loop %d/%d ...\n", loop_num, num_loops);
    boost::progress_display show_progress(lp.size());
    BOOST_FOREACH( ovd::OffsetVertex lpt, lp ) { // loop through each line/arc
      if (first) {
        first = false;
        if (!pts.find(lpt.p.x, lpt.p.y, last)){
          last = lpt.p;
          pts.add(lpt.p.x, lpt.p.y, last);
        }
      } else {
        if (pts.find(lpt.p.x, lpt.p.y, next)){
          //dprintf("WARNING: duplicate point found: (%g,%g) (%g,%g)...\n", lpt.p.x, lpt.p.y, next.x, next.y);
        } else {
          next = lpt.p;
          pts.add(lpt.p.x, lpt.p.y, next);
        }
        bool segment_fails = false;
        if (segment_intersects(segs, seg, interseg)){
          dprintf("WARNING: segment intersection found ...\n");
          if (!pts.find(interseg.first.x, interseg.first.y, interseg.first)){
            dprintf("   can't find intersecting segment's first point's id: (%g,%g) ...\n", interseg.first.x, interseg.first.y);
            segment_fails = true;
          }
          if (!pts.find(interseg.second.x, interseg.second.y, interseg.second)){
            dprintf("   can't find intersecting segment's second point's id: (%g,%g) ...\n", interseg.second.x, interseg.second.y);
            segment_fails = true;
          }
          if (!segment_fails){
            dprintf("   new segment: (%g,%g)-(%g,%g) ...\n", last.x, last.y, next.x, next.y);
            dprintf("   previous segment: (%g,%g)-(%g,%g) ...\n", interseg.first.x, interseg.first.y, interseg.second.x, interseg.second.y);
            segment_fails = true;
          }
        }
        if (!segment_fails) {
          seg.first = last;
          seg.second = next;
          segs.push_back(seg);
          last = next;
        } else {
          dprintf("WARNING: new segment was not added: (%g,%g)-(%g,%g) ...\n", last.x, last.y, next.x, next.y);
        }
      }
      ++show_progress;
  } }
  return true;
}

//bool CPocket::SanitizeOVDLoops(ovd::OffsetLoops &lps, Pts& pts, Segs& sgs){
//  /*
//  OVD assumes unique vertices and nonintersecting segments (where by
//  "intersections" we mean points which are not endpoints).
//  */
//  int nm_lps = lps.size();
//  int lp_nm = 0;
//  BOOST_FOREACH( ovd::OffsetLoop lp, lps ) {
//    dprintf("(lp %d/%d) considering lp.", lp_nm, nm_lps);
//    ovd::OffsetVertex first;
//    ovd::Point prev;
//    ovd::Point next;
//    int nm_vtxs = lp.size();
//    int vtx_nm = 0;
//    BOOST_FOREACH( ovd::OffsetVertex vtx, lp ) {
//      dprintf("(lp %d/%d) (vtx %d/%d) considering lp pt (%g,%g).", lp_num, num_lps, vtx_nm, nm_vtxs, vtx.p.x, vtx.p.y);
//      bool pt_is_duplicate = pts.find(vtx.p.x, vtx.p.y, next);
//      if (0 == vtx_nm) {
//        /* Handle first point of loop. */
//        if (pt_is_duplicate) {
//          dprintf("(lp %d/%d) (vtx %d/%d) WARN: first lp pt (%g,%g) is duplicate.", lp_num, num_lps, vtx_nm, nm_vtxs, vtx.p.x, vtx.p.y);
//        }
//        first = vtx;
//      } else {
//        if (nm_vtxs-1 == vtx_nm) {
//          /* Handle last point of loop. */
//          /*
//          Since a loop must be closed, it must start and end at the same vertex.
//          */
//          bool lp_is_closed = true;
//          if (!pt is duplicate) {
//            dprintf("(lp %d/%d) (vtx %d/%d) WARN: last lp pt (%g,%g) should be duplicate, but isn't.", lp_num, num_lps, vtx_nm, nm_vtxs, vtx.p.x, vtx.p.y);
//            lp_is_closed = false;
//          }
//          if (first.p != next) {
//            dprintf("(lp %d/%d) (vtx %d/%d) WARN: last lp pt (%g,%g) should equal first, but doesn't.", lp_num, num_lps, vtx_nm, nm_vtxs, vtx.p.x, vtx.p.y);
//            lp_is_closed = false;
//          }
//          if (!lp_is_closed) {
//            dprintf("(lp %d/%d) (vtx %d/%d) WARN: must close open lp.", lp_num, num_lps, vtx_nm, nm_vtxs);
//            lp.push_back(first);
//            nm_vtxs = lp.size();
//          }
//        }
//        /* Handle segments. */
//        Seg sg;
//        Sets intersgs;
//        intersections(sg, sgs, intersgs);
//
//      }
//      prev = next;
//      vtx_nm++;
//    }
//    lp_nm++;
//  }
//  return true;
//}

bool CPocket::AddOffsetLoopsToOVD(
  ovd::VoronoiDiagram &vd,
  ovd::OffsetLoops &loops
){
  // First load all points, and then load all lines.
  PtIDs pt_ids; // 
  Segs segs;
  SegIDs seg_ids;
  ovd::Point last_pt;
  int last_pt_id = -1;
  int next_pt_id = -1;
  Seg seg, interseg;
  SegID seg_id;

  bool first = true;
  int num_loops = loops.size();
  int loop_num = 0;
  BOOST_FOREACH( ovd::OffsetLoop lp, loops ) { // loop through each loop
    loop_num++;
    first = true;
    dprintf("adding points for loop %d/%d ...\n", loop_num, num_loops);
    boost::progress_display show_progress(lp.size());
    BOOST_FOREACH( ovd::OffsetVertex lpt, lp ) { // loop through each line/arc
      if (first) {
        first = false;
        if (!pt_ids.find(lpt.p.x, lpt.p.y, last_pt_id)){
          last_pt = lpt.p;
          last_pt_id = vd.insert_point_site(lpt.p);
          pt_ids.add(lpt.p.x, lpt.p.y, last_pt_id);
        }
      } else {
        if (pt_ids.find(lpt.p.x, lpt.p.y, next_pt_id)){
          //dprintf("WARNING: duplicate point found: (%g,%g) (id:%d)...\n", lpt.p.x, lpt.p.y, next_pt_id);
        } else {
          next_pt_id = vd.insert_point_site(lpt.p);
          pt_ids.add(lpt.p.x, lpt.p.y, next_pt_id);
        }
        bool segment_fails = false;
        if (segment_intersects(segs, seg, interseg)){
          dprintf("WARNING: segment intersection found ...\n");
          int interseg_first_id, interseg_second_id;
          if (!pt_ids.find(interseg.first.x, interseg.first.y, interseg_first_id)){
            dprintf("   can't find intersecting segment's first point's id: (%g,%g) ...\n", interseg.first.x, interseg.first.y);
            segment_fails = true;
          }
          if (!pt_ids.find(interseg.second.x, interseg.second.y, interseg_second_id)){
            dprintf("   can't find intersecting segment's second point's id: (%g,%g) ...\n", interseg.second.x, interseg.second.y);
            segment_fails = true;
          }
          if (!segment_fails){
            dprintf("   new segment: (%g,%g)-(%g,%g) (ids:%d-%d)...\n", last_pt.x, last_pt.y, lpt.p.x, lpt.p.y, last_pt_id, next_pt_id);
            dprintf("   previous segment: (%g,%g)-(%g,%g) (ids:%d-%d)...\n", interseg.first.x, interseg.first.y, interseg.second.x, interseg.second.y, interseg_first_id, interseg_second_id);
            segment_fails = true;
          }
        }
        //if (!segment_fails) {
        if (true) {
          seg.first = last_pt;
          seg.second = lpt.p;
          seg_id.first = last_pt_id;
          seg_id.second = next_pt_id;
          segs.push_back(seg);
          seg_ids.push_back(seg_id);
          last_pt = lpt.p;
          last_pt_id = next_pt_id;
        } else {
          dprintf("WARNING: new segment was not added: (%g,%g)-(%g,%g) (ids:%d-%d)...\n", last_pt.x, last_pt.y, lpt.p.x, lpt.p.y, last_pt_id, next_pt_id);
        }
      }
      ++show_progress;
  } }
  dprintf("adding lines to voronoi diagram ...\n");
  {
    boost::progress_display show_progress(seg_ids.size());
    BOOST_FOREACH( SegID seg_id, seg_ids ) { // loop through each loop
        bool success = vd.insert_line_site(seg_id.first, seg_id.second);
        if(!success) dprintf("... failed to insert line site %i-%i.\n", seg_id.first, seg_id.second);
        ++show_progress;
  } }
  return true;
}

bool CPocket::ConvertSketchesToOVD(
  std::list<HeeksObj *> &sketches,
  ovd::VoronoiDiagram &vd,
  TranslateScale &ts,
  CMachineState *pMachineState
){
  ovd::OffsetLoops loops;
  ConvertSketchesToOVDOffsetLoops(sketches, loops, pMachineState);
  GetOVDOffsetLoopsScaling(loops, ts);
  ScaleOVDOffsetLoops(loops, ts);
  AddOffsetLoopsToOVD(vd, loops);
  return true;
}

void CPocket::DetailSpan(const Span &span)
{
  dprintf("  span: is_start_span:%d, p:(%g, %g), v.type:%d, v.p:(%g, %g), v.c:(%g, %g)\n",
    span.m_start_span,
    span.m_p.x, span.m_p.y,
    span.m_v.m_type,
    span.m_v.m_p.x, span.m_v.m_p.y,
    span.m_v.m_c.x, span.m_v.m_c.y
  );
}

void CPocket::DetailVertex(const CVertex &vertex)
{
  dprintf("  vertex: type:%d, p:(%g, %g), c:(%g, %g)\n",
    vertex.m_type,
    vertex.m_p.x, vertex.m_p.y,
    vertex.m_c.x, vertex.m_c.y
  );
}

void CPocket::DetailCurve(const CCurve &curve)
{
  dprintf("entered ...\n");
  dprintf("new curve ...\n");
  dprintf("recur depth:%d\n", curve.m_recur_depth);
  dprintf("IsClosed():%d\n", curve.IsClosed());
  dprintf("IsClockwise():%d\n", curve.IsClockwise());
  dprintf("as vertices:\n");
  for(std::list<CVertex>::const_iterator v = curve.m_vertices.begin(); v != curve.m_vertices.end(); v++){
    DetailVertex(*v);
  }
  dprintf("as spans:\n");
  std::list<Span> spans;
  curve.GetSpans(spans);
  for(std::list<Span>::const_iterator s = spans.begin(); s != spans.end(); s++){
    DetailSpan(*s);
  }
  dprintf("... done.\n");
}

void CPocket::DetailArea(const CArea &area)
{
  dprintf("entered ...\n");
  for(std::list<CCurve>::const_iterator c = area.m_curves.begin(); c != area.m_curves.end(); c++){
    dprintf("new curve ...\n");
    DetailCurve(*c);
  }
  dprintf("... done.\n");
}

bool CPocket::ConvertOVDLoopToSketch(
  ovd::OffsetLoop &loop,
  HeeksObj *sketch,
  double z
) {
  double previous[3], p[3], c[3], u[3];
  ovd::Point previous_pt;
  int n = 0;
  BOOST_FOREACH(ovd::OffsetVertex ofv, loop){
    previous[0] = p[0]; previous[1] = p[1]; previous[2] = p[2];
    p[0] = ofv.p.x; p[1] = ofv.p.y; p[2] = z;
    if(0 < n){
      if((ofv.r == -1.) || ((ofv.p - previous_pt).norm() <= 0.01)){
        // Line, or an arc so tiny we should treat it as a line.
        sketch->Add(heeksCAD->NewLine(previous, p), NULL);
      } else {
        // Arc.
        c[0] = ofv.c.x; c[1] = ofv.c.y; c[2] = z;
        // 1: counterclockwise; -1: clockwise
        u[0] = u[1] = 0; u[2] = (ofv.cw)?(1):(-1);
        sketch->Add(heeksCAD->NewArc(previous, p, c, u), NULL);
      }
    }
    n++;
  }
  return true;
}

bool CPocket::ConvertOVDLoopsToSketches(
  ovd::OffsetLoops &loops,
  std::list<HeeksObj *> &sketches,
  double z
) {
  BOOST_FOREACH(ovd::OffsetLoop loop, loops){
    HeeksObj *sketch = heeksCAD->NewSketch();
    ConvertOVDLoopToSketch(loop, sketch, z);
    sketches.push_back(sketch);
  }
  return true;
}

bool CPocket::ConvertCurveToOVDLoop(CCurve& curve, ovd::OffsetLoop& loop)
{
  for(std::list<CVertex>::iterator v = curve.m_vertices.begin(); v != curve.m_vertices.end(); v++){
    ovd::Point p(v->m_p.x, v->m_p.y);
    if(0 == v->m_type){
    // Line.
      loop.push_back(ovd::OffsetVertex(p));
    } else {
    // Arc.
      ovd::Point c(v->m_c.x, v->m_c.y);
      bool cw(v->m_type == -1);
      double r((p-c).norm());
      loop.push_back(ovd::OffsetVertex(p, r, c, cw));
    }
  }
  return true;
}

void AppendOVDVertexToCurve(CCurve& curve, ovd::OffsetVertex& v)
{
  if (v.r == -1) { curve.append(CVertex(0, Point(v.p.x, v.p.y), Point())); }
  // Arc.
  else {
    int span_type = (v.cw)?(-1):(1);
    curve.append(CVertex(span_type, Point(v.p.x, v.p.y), Point(v.c.x, v.c.y)));
  }
}

bool CPocket::ConvertOVDLoopToCurve(CCurve& curve, ovd::OffsetLoop& loop)
{
  BOOST_FOREACH(ovd::OffsetVertex v, loop) { AppendOVDVertexToCurve(curve, v); }
  if (0 < loop.size()) {
    ovd::OffsetVertex v = *(loop.begin());
    AppendOVDVertexToCurve(curve, v);
  }
  return true;
}

bool CPocket::ConvertAreaToOVDLoops(CArea& area, ovd::OffsetLoops& loops)
{
  for(std::list<CCurve>::iterator c = area.m_curves.begin(); c != area.m_curves.end(); c++){
    ovd::OffsetLoop loop;
    ConvertCurveToOVDLoop(*c, loop);
    loops.push_back(loop);
  }
  return true;
}

bool CPocket::ConvertOVDLoopsToArea(CArea& area, ovd::OffsetLoops &loops)
{
  BOOST_FOREACH(ovd::OffsetLoop loop, loops){
    CCurve curve;
    ConvertOVDLoopToCurve(curve, loop);
    area.append(curve);
  }
  return true;
}

bool CPocket::ConvertCurveToSketch(CCurve& curve, HeeksObj *sketch, CMachineState *pMachineState, double z)
{
  std::list<Span> spans;
  curve.GetSpans(spans);
  int num_spans = spans.size();
  int span_num = 0;
  for(std::list<Span>::iterator s = spans.begin(); s != spans.end(); s++){
    span_num++;
    double start[3]; start[0] = s->m_p.x; start[1] = s->m_p.y; start[2] = z;
    double end[3]; end[0] = s->m_v.m_p.x; end[1] = s->m_v.m_p.y; end[2] = z;
    // Line.
    if(0 == s->m_v.m_type){ sketch->Add(heeksCAD->NewLine(start, end), NULL); }
    // Arc.
    else {
      dprintf("(span %d/%d) span is an arc ...\n", span_num, num_spans);
      double centre[3]; centre[0] = s->m_v.m_c.x; centre[1] = s->m_v.m_c.y; centre[2] = z;
      double up[3]; up[0] = up[1] = 0; up[2] = s->m_v.m_type; // 1: counterclockwise; -1: clockwise
      sketch->Add(heeksCAD->NewArc(start, end, centre, up), NULL);
    }
  }
  return true;
}

bool CPocket::ConvertAreaToSketches(CArea& area, std::list<HeeksObj *> &sketches, CMachineState *pMachineState, double z)
{
  dprintf("entered ...\n");
  dprintf("z: %g ...\n", z);
  dprintf("converting curves to sketches ...\n");
  dprintf("area.m_curves.size(): %d ...\n", int(area.m_curves.size()));
  for(std::list<CCurve>::iterator c = area.m_curves.begin(); c != area.m_curves.end(); c++){
    HeeksObj *sketch = heeksCAD->NewSketch();
    ConvertCurveToSketch(*c, sketch, pMachineState, z);
    sketches.push_back(sketch);
  }
  dprintf("... done converting curves to sketches.\n");
  dprintf("... done.\n");
  return true;
}
