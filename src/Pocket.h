// Pocket.h
/*
 * Copyright (c) 2009, Dan Heeks
 * This program is released under the BSD license. See the file COPYING for
 * details.
 */

#include "HeeksCNCTypes.h"
#include "DepthOp.h"
#include "CTool.h"

#include "openvoronoi/offset.hpp"
#include "openvoronoi/voronoidiagram.hpp"

#include <map>
#include <utility>

class CPocket;
class CArea;
class CCurve;
class CVertex;
class Span;
struct TranslateScale;

struct Pts {
    typedef std::map<double, ovd::Point> _PtMap;
    typedef _PtMap::iterator _PtMapIter;
    typedef std::map<double, _PtMap> PtMap;
    typedef PtMap::iterator PtMapIter;
    PtMap m_pts;
    void add(double a, double b, ovd::Point& p){
        m_pts[a][b] = p;
    }
    bool find(double a, double b, ovd::Point &p){
        PtMapIter _pts(m_pts.find(a));
        if (_pts == m_pts.end()) return false;
        _PtMapIter pts(m_pts[a].find(b));
        if (pts == m_pts[a].end()) return false;
        p = m_pts[a][b];
        return true;
    }
};

struct PtIDs {
    typedef std::map<double, int> IDMap;
    typedef IDMap::iterator IDMapIter;
    typedef std::map<double, IDMap> PtIDMap;
    typedef PtIDMap::iterator PtIDMapIter;
    PtIDMap pt_ids;
    void add(double a, double b, int id){
        pt_ids[a][b] = id;
    }
    bool find(double a, double b, int &result){
        PtIDMapIter ids(pt_ids.find(a));
        if (ids == pt_ids.end()) return false;
        IDMapIter id(pt_ids[a].find(b));
        if (id == pt_ids[a].end()) return false;
        result = pt_ids[a][b];
        return true;
    }
};

typedef std::pair<int, int> SegID;
typedef std::vector<SegID> SegIDs;
typedef std::pair<ovd::Point, ovd::Point> Seg;
typedef std::vector<Seg> Segs;

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

	static bool SketchNeedsReordering(SketchOrderType order);

    static void DetailSpan(const Span &span);
    static void DetailVertex(const CVertex &vertex);
    static void DetailCurve(const CCurve &curve);
    static void DetailArea(const CArea &area);

    // LibAREA conversion functions
    static int ConvertSketchToArea(HeeksObj *skch, CArea&, CMachineState*);
	static bool ConvertSketchesToArea(std::list<HeeksObj *> &skchs, CArea&, CMachineState*);
	static bool ConvertCurveToSketch(CCurve&, HeeksObj *skch, CMachineState*, double z=0);
	static bool ConvertAreaToSketches(CArea&, std::list<HeeksObj *> &skchs, CMachineState*, double z=0);

    static void OVDArcToLines(ovd::Point&, ovd::OffsetVertex&, ovd::OffsetLoop&, int steps_per_circle=0, double ds=0.1);
    static ovd::OffsetLoop OVDLoopArcsToLines(ovd::OffsetLoop&, int steps_per_circle=0, double ds=0.1);

	static bool ConvertSketchesToOVDOffsetLoops(std::list<HeeksObj *> &skchs, ovd::OffsetLoops&, CMachineState*);
	static bool GetOVDOffsetLoopScaling(ovd::OffsetLoop&, TranslateScale &);
	static bool GetOVDOffsetLoopsScaling(ovd::OffsetLoops&, TranslateScale &);
	static bool ScaleOVDOffsetLoops(ovd::OffsetLoops&, TranslateScale &);
	static bool InvScaleOVDOffsetLoops(ovd::OffsetLoops&, TranslateScale &);

    static bool OVDFilterDuplicates(ovd::OffsetLoops&, Pts&, Segs&);
    static bool SanitizeOVDLoops(ovd::OffsetLoops&, Pts&, Segs&);

	static bool AddOffsetLoopsToOVD(ovd::VoronoiDiagram&, ovd::OffsetLoops&);
	static bool ConvertSketchesToOVD(std::list<HeeksObj *> &skchs, ovd::VoronoiDiagram&, TranslateScale&, CMachineState*);
    static bool ConvertOVDLoopToSketch(ovd::OffsetLoop&, HeeksObj *skch, double z);
    static bool ConvertOVDLoopsToSketches(ovd::OffsetLoops&, std::list<HeeksObj *> &skchs, double z);

	static bool ConvertCurveToOVDLoop(CCurve&, ovd::OffsetLoop&);
	static bool ConvertOVDLoopToCurve(CCurve&, ovd::OffsetLoop&);
	static bool ConvertAreaToOVDLoops(CArea&, ovd::OffsetLoops&);
	static bool ConvertOVDLoopsToArea(CArea&, ovd::OffsetLoops&);

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
    : first(true)
    , bits(0)
    , bits_power(1.)
    , inv_bits_power(1.)
    {}
    bool first;
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

