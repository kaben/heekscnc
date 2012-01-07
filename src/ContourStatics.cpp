#include "Contour.h"
#include <BRepAdaptor_Curve.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <TopoDS.hxx>

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

/* static */ std::vector<TopoDS_Edge> CContour::SortEdges( const TopoDS_Wire & wire )
{
    std::vector<TopoDS_Edge> edges;

	for(BRepTools_WireExplorer expEdge(TopoDS::Wire(wire)); expEdge.More(); expEdge.Next())
	{
	    edges.push_back( TopoDS_Edge(expEdge.Current()) );
	} // End for

	for (std::vector<TopoDS_Edge>::iterator l_itEdge = edges.begin(); l_itEdge != edges.end(); l_itEdge++)
    {
        if (l_itEdge == edges.begin())
        {
            // It's the first edge.  Find the edge whose endpoint is closest to gp_Pnt(0,0,0) so that
            // the resutls of this sorting are consistent.  When we just use the first edge in the
            // wire, we end up with different results every time.  We want consistency so that, if we
            // use this Contour operation as a location for drilling a relief hole (one day), we want
            // to be sure the machining will begin from a consistently known location.

            std::vector<TopoDS_Edge>::iterator l_itStartingEdge = edges.begin();
            gp_Pnt closest_point = _GetStart(*l_itStartingEdge);
            if (_GetEnd(*l_itStartingEdge).Distance(gp_Pnt(0,0,0)) < closest_point.Distance(gp_Pnt(0,0,0)))
            {
                closest_point = _GetEnd(*l_itStartingEdge);
            }
            for (std::vector<TopoDS_Edge>::iterator l_itCheck = edges.begin(); l_itCheck != edges.end(); l_itCheck++)
            {
                if (_GetStart(*l_itCheck).Distance(gp_Pnt(0,0,0)) < closest_point.Distance(gp_Pnt(0,0,0)))
                {
                    closest_point = _GetStart(*l_itCheck);
                    l_itStartingEdge = l_itCheck;
                }

                if (_GetEnd(*l_itCheck).Distance(gp_Pnt(0,0,0)) < closest_point.Distance(gp_Pnt(0,0,0)))
                {
                    closest_point = _GetEnd(*l_itCheck);
                    l_itStartingEdge = l_itCheck;
                }
            }

            EdgeComparison compare( *l_itStartingEdge );
            std::sort( edges.begin(), edges.end(), compare );
        } // End if - then
        else
        {
            // We've already begun.  Just sort based on the previous point's location.
            std::vector<TopoDS_Edge>::iterator l_itNextEdge = l_itEdge;
            l_itNextEdge++;

            if (l_itNextEdge != edges.end())
            {
                EdgeComparison compare( *l_itEdge );
                std::sort( l_itNextEdge, edges.end(), compare );
            } // End if - then
        } // End if - else
    } // End for

    return(edges);

} // End SortEdges() method


/**
	Find the edge whose endpoint is closest to gp_Pnt(0,0,0), and then traverse
	the rest of the wire loop starting at this edge, building a new edge list
	as we go. We do this so that the results of this sorting are consistent
	from one SortEdges2() call to the next. We want consistency so
	that, if we use this Contour operation as a location for drilling a relief
	hole (one day), we want to be sure the machining will begin from a
	consistently known location.

  Note: this goal isn't actually met in either SortEdges() or SortEdges2(). See
  unit tests for details and demonstration.
*/
/* static */ std::vector<TopoDS_Edge> CContour::SortEdges2( const TopoDS_Wire & wire )
{
    std::vector<TopoDS_Edge> edges;

    for(BRepTools_WireExplorer expEdge(TopoDS::Wire(wire)); expEdge.More(); expEdge.Next())
    {
        edges.push_back( TopoDS_Edge(expEdge.Current()) );
    } // End for

    std::vector<TopoDS_Edge> final_edges;
    EdgeComparisonRefPt compare_ref_origin(gp_Pnt(0,0,0));
    std::vector<TopoDS_Edge>::iterator edge_closest_to_origin(
      std::min_element(
        edges.begin(), edges.end(), compare_ref_origin
      )
    );
    std::vector<TopoDS_Edge>::iterator l_itEdge(edge_closest_to_origin);
    while ( true )
    {
        final_edges.push_back( *l_itEdge );
        l_itEdge++;
        if ( l_itEdge == edges.end() )
        {
          l_itEdge = edges.begin();
        }
        if ( l_itEdge == edge_closest_to_origin )
        {
          break;
        }
    } // End while

    return(final_edges);

} // End SortEdges2() method


