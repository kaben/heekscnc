#include "Contour.h"
#include "interface/TestMacros.h"
#include <BRepAdaptor_Curve.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <TopoDS.hxx>

#include "interface/TestMacros.h"

/* static */ gp_Pnt CContour::_GetStart(const TopoDS_Edge &edge)
{
    BRepAdaptor_Curve curve(edge);
    double uStart = curve.FirstParameter();
    gp_Pnt PS;
    gp_Vec VS;
    curve.D1(uStart, PS, VS);

    return(PS);
}

/* static */ gp_Pnt CContour::_GetEnd(const TopoDS_Edge &edge)
{
    BRepAdaptor_Curve curve(edge);
    double uEnd = curve.LastParameter();
    gp_Pnt PE;
    gp_Vec VE;
    curve.D1(uEnd, PE, VE);

    return(PE);
}

/**
	Find the edge whose endpoint is closest to gp_Pnt(0,0,0), and then traverse
	the rest of the wire loop starting at this edge, building a new edge list
	as we go. We do this so that the results of this sorting are consistent
	from one SortEdges() call to the next. We want consistency so
	that, if we use this Contour operation as a location for drilling a relief
	hole (one day), we want to be sure the machining will begin from a
	consistently known location.
*/
/* static */ std::vector<TopoDS_Edge> CContour::SortEdges( const TopoDS_Wire & wire )
{
    std::vector<TopoDS_Edge> edges;

	for(BRepTools_WireExplorer expEdge(TopoDS::Wire(wire)); expEdge.More(); expEdge.Next())
	{
	    edges.push_back( TopoDS_Edge(expEdge.Current()) );
	}

    std::partial_sort(
        edges.begin(),
        edges.begin()+1,
        edges.end(),
        EdgeComparisonRefPt(gp_Pnt(0,0,0))
    );
    int num_edges = edges.size();
    int edge_num = 0;
	for (std::vector<TopoDS_Edge>::iterator l_itEdge = edges.begin(); l_itEdge != edges.end(); l_itEdge++)
    {
        edge_num++;
        dprintf("(edge %d/%d) ...\n", edge_num, num_edges);
        EdgeComparison compare( *l_itEdge );
        std::partial_sort( l_itEdge+1, l_itEdge+2, edges.end(), compare );
    }

    return(edges);

} // End SortEdges() method


