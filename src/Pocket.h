// Pocket.h
/*
 * Copyright (c) 2009, Dan Heeks
 * This program is released under the BSD license. See the file COPYING for
 * details.
 */

#include "HeeksCNCTypes.h"
#include "DepthOp.h"
#include "CTool.h"

namespace ovd {
    class VoronoiDiagram;
    class Point;
};

class CPocket;
class CArea;
class CCurve;
class CVertex;
class Span;
struct TranslateScale;

class CPocketParams{
public:
	int m_starting_place;
	double m_material_allowance;
	double m_step_over;
	bool m_keep_tool_down_if_poss;
	bool m_use_zig_zag;
	double m_zig_angle;
	bool m_zig_unidirectional;

	typedef enum {
		eConventional,
		eClimb
	}eCutMode;
	eCutMode m_cut_mode;

    typedef enum {
            ePlunge = 0,
            eRamp,
            eHelical,
            eUndefinedeDescentStrategy
    } eEntryStyle;
    eEntryStyle m_entry_move;

	CPocketParams();

    void set_initial_values(const CTool::ToolNumber_t tool_number);
	void GetProperties(CPocket* parent, std::list<Property *> *list);
	void WriteXMLAttributes(TiXmlNode* pElem);
	void ReadFromXMLElement(TiXmlElement* pElem);
	static wxString ConfigScope() { return(_T("Pocket")); }

	bool operator== ( const CPocketParams & rhs ) const;
	bool operator!= ( const CPocketParams & rhs ) const { return(! (*this == rhs)); }
};

class CPocket: public CDepthOp{
public:
	typedef std::list<int> Sketches_t;
	Sketches_t m_sketches;
	CPocketParams m_pocket_params;

	static double max_deviation_for_spline_to_arc;

	CPocket():CDepthOp(GetTypeString(), 0, PocketType){}
	CPocket(const std::list<int> &sketches, const int tool_number );
	CPocket(const std::list<HeeksObj *> &sketches, const int tool_number );
	CPocket( const CPocket & rhs );
	CPocket & operator= ( const CPocket & rhs );

	bool operator== ( const CPocket & rhs ) const;
	bool operator!= ( const CPocket & rhs ) const { return(! (*this == rhs)); }

	// HeeksObj's virtual functions
	int GetType()const{return PocketType;}
	const wxChar* GetTypeString(void)const{return _T("Pocket");}
	void glCommands(bool select, bool marked, bool no_color);
	const wxBitmap &GetIcon();
	void GetProperties(std::list<Property *> *list);
	HeeksObj *MakeACopy(void)const;
	void CopyFrom(const HeeksObj* object);
	void WriteXML(TiXmlNode *root);
	bool CanAddTo(HeeksObj* owner);
	void GetTools(std::list<Tool*>* t_list, const wxPoint* p);
#ifdef OP_SKETCHES_AS_CHILDREN
	void ReloadPointers();
#endif
	void GetOnEdit(bool(**callback)(HeeksObj*));
	bool Add(HeeksObj* object, HeeksObj* prev_object);

	// COp's virtual functions
	Python AppendTextToProgram(CMachineState *pMachineState);
	void WriteDefaultValues();
	void ReadDefaultValues();

	static HeeksObj* ReadFromXMLElement(TiXmlElement* pElem);

	std::list<wxString> DesignRulesAdjustment(const bool apply_changes);

	static void GetOptions(std::list<Property *> *list);
	static void ReadFromConfig();
	static void WriteToConfig();

    static void DetailSpan(const Span &span);
    static void DetailVertex(const CVertex &vertex);
    static void DetailCurve(const CCurve &curve);
    static void DetailArea(const CArea &area);
    // LibAREA conversion functions
	static bool ConvertSketchesToArea(
      std::list<HeeksObj *> &sketches,
      CArea &area,
      CMachineState *pMachineState
    );
	static bool ConvertSketchesToOVD(
      std::list<HeeksObj *> &sketches,
      ovd::VoronoiDiagram &vd,
      TranslateScale &ts,
      CMachineState *pMachineState
    );

	static bool ConvertCurveToSketch(
      CCurve& curve,
      HeeksObj *sketch,
      CMachineState *pMachineState,
      double z = 0.
    );
	static bool ConvertAreaToSketches(
      CArea& area,
      std::list<HeeksObj *> &sketches,
      CMachineState *pMachineState,
      double z = 0.
    );

    /* Possibly implement, if they look like they're needed. */
    //static bool ConvertOVDMedialPointListToSketch(
    //  ovd::MedialPointList,
    //  TranslateScale &ts,
    //  std::list<HeeksObj *> &sketches,
    //  CMachineState *pMachineState,
    //  double z = 0.
    //);
    //static bool ConvertOVDChainToSketches(
    //  ovd::Chain,
    //  TranslateScale &ts,
    //  std::list<HeeksObj *> &sketches,
    //  CMachineState *pMachineState,
    //  double z = 0.
    //);
	//static bool ConvertFaceToArea(const TopoDS_Face& face, CArea& area, double deviation);
	//static bool ConvertWireToArea(const TopoDS_Wire& wire, CArea& area, double deviation);
	//static bool ConvertEdgeToArea(const TopoDS_Edge& edge, CArea& area, double deviation);
	//static bool ConvertAreaToFace(const CArea& area, std::list<TopoDS_Shape> &face);
	//static bool ConvertAreaToWire(const CArea& area, std::list<TopoDS_Shape> &wire);
};

struct TranslateScale {
    TranslateScale()
    : bits(0)
    , bits_power(1.)
    , inv_bits_power(1.)
    {}
    double bits;
    double bits_power;
    double inv_bits_power;
    double min_x, min_y;
    double max_x, max_y;
    double d_x, d_y;
    double c_x, c_y;
    double s, is;
    double s_x, s_y;
    double is_x, is_y;
    void set_bits(int _bits);
    void set(ovd::Point &p);
    void update(ovd::Point &p);
    void set_translate_scale();
    void set_translate_scale_fixed_aspect();
    void translate(ovd::Point &p);
    void scale(double &d);
    void scale(ovd::Point &p);
    void translate_scale(ovd::Point &p);
    void inv_scale(ovd::Point &p);
    void inv_scale(double &d);
    void inv_translate(ovd::Point &p);
    void inv_scale_translate(ovd::Point &p);
    void round(double &d);
    void round(ovd::Point &p);
};

