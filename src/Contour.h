
#ifndef CONTOUR_CYCLE_CLASS_DEFINITION
#define CONTOUR_CYCLE_CLASS_DEFINITION

// Contour.h
/*
 * Copyright (c) 2009, Dan Heeks, Perttu Ahola
 * This program is released under the BSD license. See the file COPYING for
 * details.
 */
#ifndef STABLE_OPS_ONLY

#ifndef DEPTH_OP_HEADER
#include "DepthOp.h"
#endif
#ifndef HeeksCNCTypes_h
#include "HeeksCNCTypes.h"
#endif
#ifndef _GLIBCXX_ALGORITHM
#include <algorithm>
#endif
#ifndef _GLIBCXX_FUNCTIONAL
#include <functional>
#endif
#ifndef _GLIBCXX_LIST
#include <list>
#endif
#ifndef _GLIBCXX_VECTOR
#include <vector>
#endif
#ifndef CNCPoint_h
#include "CNCPoint.h"
#endif
#ifndef _gp_Pnt_HeaderFile
#include <gp_Pnt.hxx>
#endif
#ifndef _TopoDS_Wire_HeaderFile
#include <TopoDS_Wire.hxx>
#endif
#ifndef _TopoDS_Edge_HeaderFile
#include <TopoDS_Edge.hxx>
#endif


class CContour;

class CContourParams{
public:
	typedef enum {
		eRightOrInside = -1,
		eOn = 0,
		eLeftOrOutside = +1
	}eSide;
	eSide m_tool_on_side;

	typedef enum
	{
	    ePlunge = 0,
	    eRamp
    } EntryMove_t;
    EntryMove_t m_entry_move_type;

public:
	CContourParams()
	{
		m_tool_on_side = eOn;
		m_entry_move_type = ePlunge;
		ReadDefaultValues();
	}

	void WriteDefaultValues();
    void ReadDefaultValues();
	void GetProperties(CContour* parent, std::list<Property *> *list);
	void WriteXMLAttributes(TiXmlNode* pElem);
	void ReadParametersFromXMLElement(TiXmlElement* pElem);

	const wxString ConfigPrefix(void)const{return _T("Contour");}

	bool operator== ( const CContourParams & rhs ) const
	{
		return(m_tool_on_side == rhs.m_tool_on_side);
	}

	bool operator!= ( const CContourParams & rhs ) const
	{
		return(! (*this == rhs));	// Call the equivalence operator.
	}

    friend wxString & operator<< ( wxString & str, CContourParams::EntryMove_t & move )
    {
        switch(move)
        {
            case CContourParams::ePlunge:
                str << _("Plunge");
                break;

            case CContourParams::eRamp:
                str << _("Ramp");
                break;
        }

        return(str);
    }
};


/**
	The CContour class is suspiciously similar to the CProfile class.  The main difference is that the NC path
	is generated from this class itself (in C++) rather than by using the kurve Pythin library.  It also
	has depth values that are RELATIVE to the sketch coordinate rather than assuming a horozontal sketch.  The idea
	of this is to allow for a rotation of the XZ or YZ planes.

	Finally, it is hoped that the CContour class will support 'bridging tabs' during a cutout so that the
	workpiece is held in place until the very last moment.

	This class uses the offet functionality for TopoDS_Wire objects to handle the path generation. This is DIFFERENT
	to that used by the Profile class in that it doesn't handle the case where the user wants to machine inside a
	sketch but where the diamter of the  tool makes that impossible.  With the Profile class, this would be
	possible for a converging sketch shape such that the tool would penetrate as far as it could without gouging
	the sketch but would not cut out the whole sketch shape.  This class allows the FAILURE to occur rather than
	allowing half the sketch to be machined.  At the initial time of writing, I consider this to be a GOOD thing.
	I wish to do some 'inlay' work and I want to know whether the  tools will COMPLETELY cut out the sketch
	shapes.  Perhaps we will add a flag to enable/disable this behaviour later.
 */

class CContour: public CDepthOp {
public:
	/**
		Define some data structures to hold references to CAD elements.
 	 */
	typedef int SymbolType_t;
	typedef unsigned int SymbolId_t;
	typedef std::pair< SymbolType_t, SymbolId_t > Symbol_t;
	typedef std::list< Symbol_t > Symbols_t;

public:
	//	These are references to the CAD elements whose position indicate where the Drilling Cycle begins.
	//	If the m_params.m_sort_drilling_locations is false then the order of symbols in this list should
	//	be respected when generating GCode.  We will, eventually, allow a user to sort the sub-elements
	//	visually from within the main user interface.  When this occurs, the change in order should be
	//	reflected in the ordering of symbols in the m_symbols list.

	Symbols_t m_symbols;
	CContourParams m_params;
	static double max_deviation_for_spline_to_arc;

	//	Constructors.
	CContour():CDepthOp(GetTypeString(), 0, ContourType)
	{
	    ReadDefaultValues();
	}

	CContour(	const Symbols_t &symbols,
			const int tool_number )
		: CDepthOp(GetTypeString(), NULL, tool_number, ContourType), m_symbols(symbols)
	{
	    ReadDefaultValues();
		ReloadPointers();
	}

	CContour( const CContour & rhs );
	CContour & operator= ( const CContour & rhs );

	// HeeksObj's virtual functions
	int GetType()const{return ContourType;}
	const wxChar* GetTypeString(void)const{return _T("Contour");}
	void glCommands(bool select, bool marked, bool no_color);

	const wxBitmap &GetIcon();
	void GetProperties(std::list<Property *> *list);
	HeeksObj *MakeACopy(void)const;
	void CopyFrom(const HeeksObj* object);
	void WriteXML(TiXmlNode *root);
	bool CanAddTo(HeeksObj* owner);
	bool CanAdd(HeeksObj *object);
	void GetTools(std::list<Tool*>* t_list, const wxPoint* p);

	void WriteDefaultValues();
    void ReadDefaultValues();

	bool operator== ( const CContour & rhs ) const;
	bool operator!= ( const CContour & rhs ) const { return(! (*this == rhs)); }
	bool IsDifferent(HeeksObj* other);

	// This is the method that gets called when the operator hits the 'Python' button.  It generates a Python
	// program whose job is to generate RS-274 GCode.
	Python AppendTextToProgram( CMachineState *pMachineState );

	static HeeksObj* ReadFromXMLElement(TiXmlElement* pElem);

	void AddSymbol( const SymbolType_t type, const SymbolId_t id ) { m_symbols.push_back( Symbol_t( type, id ) ); }

	std::list<wxString> DesignRulesAdjustment(const bool apply_changes);

	static Python GeneratePathFromWire( 	const TopoDS_Wire & wire,
											CMachineState *pMachineState,
											const double clearance_height,
											const double rapid_down_to_height,
											const double start_depth,
											const CContourParams::EntryMove_t entry_move_type );

	static Python GeneratePathForEdge(		const TopoDS_Edge &edge,
											const double first_parameter,
											const double last_parameter,
											const bool forwards,
											CMachineState *pMachineState,
											const double end_z );

    static Python GenerateRampedEntry(      ::size_t starting_edge_offset,
                                            std::vector<TopoDS_Edge> & edges,
                                            CMachineState *pMachineState,
											const double end_z );

	static bool Clockwise( const gp_Circ & circle );
	void ReloadPointers();
	static void GetOptions(std::list<Property *> *list);

	static std::vector<TopoDS_Edge> SortEdges( const TopoDS_Wire & wire );
  static std::vector<TopoDS_Edge> SortEdges2( const TopoDS_Wire & wire );
	static bool DirectionTowarardsNextEdge( const TopoDS_Edge &from, const TopoDS_Edge &to );

	static std::vector<TopoDS_Edge>::size_type NextOffset(	const std::vector<TopoDS_Edge> &edges,
													const std::vector<TopoDS_Edge>::size_type edges_offset,
													const int direction );

	static bool EdgesJoin( const TopoDS_Edge &a, const TopoDS_Edge &b );

public:
	  static CNCPoint GetStart(const TopoDS_Edge &edge) { return CContour::_GetStart(edge); }
    static CNCPoint GetEnd(const TopoDS_Edge &edge) { return CContour::_GetEnd(edge); }
	  static gp_Pnt _GetStart(const TopoDS_Edge &edge);
    static gp_Pnt _GetEnd(const TopoDS_Edge &edge);
    static double GetLength(const TopoDS_Edge &edge);
};


struct EdgeComparison : public std::binary_function<const TopoDS_Edge &, const TopoDS_Edge &, bool >
{

    gp_Pnt m_reference_start;
    gp_Pnt m_reference_end;

    EdgeComparison( const TopoDS_Edge & edge )
	  : m_reference_start(CContour::_GetStart(edge))
	  , m_reference_end(CContour::_GetEnd(edge))
    { }

    bool operator()( const TopoDS_Edge & lhs, const TopoDS_Edge & rhs ) const
    {

        gp_Pnt lhs_start(CContour::_GetStart(lhs));
        gp_Pnt lhs_end(CContour::_GetEnd(lhs));
        gp_Pnt rhs_start(CContour::_GetStart(rhs));
        gp_Pnt rhs_end(CContour::_GetEnd(rhs));

        double lhs_distance = lhs_start.Distance(m_reference_start);
        double tmp_distance = lhs_start.Distance(m_reference_end);
        if (tmp_distance < lhs_distance) lhs_distance = tmp_distance;
        tmp_distance = lhs_end.Distance(m_reference_start);
        if (tmp_distance < lhs_distance) lhs_distance = tmp_distance;
        tmp_distance = lhs_end.Distance(m_reference_end);
        if (tmp_distance < lhs_distance) lhs_distance = tmp_distance;
        
        double rhs_distance = rhs_start.Distance(m_reference_start);
        tmp_distance = rhs_start.Distance(m_reference_end);
        if (tmp_distance < rhs_distance) rhs_distance = tmp_distance;
        tmp_distance = rhs_end.Distance(m_reference_start);
        if (tmp_distance < rhs_distance) rhs_distance = tmp_distance;
        tmp_distance = rhs_end.Distance(m_reference_end);
        if (tmp_distance < rhs_distance) rhs_distance = tmp_distance;
        
        return(lhs_distance < rhs_distance);
    }
};

struct EdgeComparisonRefPt : public std::binary_function<const TopoDS_Edge &, const TopoDS_Edge &, bool >
{
    gp_Pnt m_reference_point;
    EdgeComparisonRefPt(const gp_Pnt & point): m_reference_point(point) {}

    bool operator()( const TopoDS_Edge & lhs, const TopoDS_Edge & rhs ) const
    {
        double lhs_distance = CContour::_GetStart(lhs).Distance(m_reference_point);
        double rhs_distance = CContour::_GetStart(rhs).Distance(m_reference_point);
        double tmp_distance = CContour::_GetEnd(lhs).Distance(m_reference_point);
        if (tmp_distance < lhs_distance) lhs_distance = tmp_distance;
        tmp_distance = CContour::_GetEnd(rhs).Distance(m_reference_point);
        if (tmp_distance < rhs_distance) rhs_distance = tmp_distance;
        return(lhs_distance < rhs_distance);
    }
};


#endif

#endif // CONTOUR_CYCLE_CLASS_DEFINITION
