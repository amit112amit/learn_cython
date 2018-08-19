#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned,K> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
typedef CGAL::Delaunay_triangulation_2<K,Tds> Delaunay;
typedef Delaunay::Face_circulator Face_circulator;
typedef Delaunay::Face_handle Face_handle;
typedef Delaunay::Point Point;
typedef Eigen::Vector3d Vector3d;
typedef Eigen::VectorXd VectorXd;
typedef Eigen::Matrix3d Matrix3d;
typedef Eigen::Matrix3Xd Matrix3Xd;
typedef Eigen::Map<Matrix3Xd> MapM3Xd;
typedef Eigen::Ref<Matrix3Xd> RefM3Xd;

// Pass the number of points in point cloud and address of the 0th point coords
static double_t pointCloudVolume( std::size_t N, double_t* p ){

    // We will extract 3*N doubles representing particle positions
    MapM3Xd Xt( p, 3, N );
    Matrix3Xd X0(3,N);

    // Project the points to a unit sphere
    X0 = Xt.colwise().normalized();

    // Rotate all points of the shell so that the 0th point is along z-axis
    Vector3d c = X0.col(0);
    double_t cos_t = c(2);
    double_t sin_t = std::sin( std::acos( cos_t ) );
    Vector3d axis;
    axis << c(1), -c(0), 0.;
    axis.normalize();
    Matrix3d rotMat, axis_cross, outer;
    axis_cross << 0. , -axis(2), axis(1),
               axis(2), 0., -axis(0),
               -axis(1), axis(0), 0.;
    outer.noalias() = axis*axis.transpose();
    rotMat = cos_t*Matrix3d::Identity() + sin_t*axis_cross + (1 - cos_t)*outer;
    Matrix3Xd rPts(3,N);
    rPts = rotMat*X0;

    // Calculate the stereographic projections
    Vector3d p0;
    p0 << 0,0,-1.0; // Point on the plane of projection
    c = rPts.col(0); // The point from which we are projecting

    MapM3Xd l0( &(rPts(0,1)), 3, N-1 );
    Matrix3Xd l(3,N-1), proj(3,N-1);
    l = (l0.colwise() - c).colwise().normalized(); // dirns of projections
    for( std::size_t j=0; j < N-1; ++j ){
        proj.col(j) = ((p0(2) - l0(2,j))/l(2,j))*l.col(j) + l0.col(j);
    }

    // Insert the projected points in a CGAL vertex_with_info vector
    std::vector< std::pair< Point, unsigned> > verts;
    for( std::size_t j=0; j < N-1; ++j ){
        verts.push_back(std::make_pair(Point(proj(0,j),proj(1,j)),j+1));
    }

    // Triangulate
    Delaunay dt( verts.begin(), verts.end() );

    // Iterate over the triangles to calculate volume
    double_t volume = 0.0;
    for( auto ffi = dt.finite_faces_begin(); ffi != dt.finite_faces_end();
            ++ffi){
        auto i = ffi->vertex(0)->info();
        auto j = ffi->vertex(2)->info();
        auto k = ffi->vertex(1)->info();
        volume += 0.166666667*(Xt.col(i).dot(Xt.col(j).cross(Xt.col(k))));
    }

    // Iterate over infinite faces
    Face_circulator fc = dt.incident_faces(dt.infinite_vertex()), done(fc);
    if (fc != 0) {
        do{
            auto i = dt.is_infinite(fc->vertex(0))?0:fc->vertex(0)->info();
            auto j = dt.is_infinite(fc->vertex(2))?0:fc->vertex(2)->info();
            auto k = dt.is_infinite(fc->vertex(1))?0:fc->vertex(1)->info();
            volume += 0.166666667*
                (Xt.col(i).dot(Xt.col(j).cross(Xt.col(k))));
        }while(++fc != done);
    }

    return volume;
}
