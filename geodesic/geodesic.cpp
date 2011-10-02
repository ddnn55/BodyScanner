#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/pcd_io.h> // for toRosMsg
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <cstdio>
#include <sys/time.h>

template <typename PointType>
float distance(PointType a, PointType b) {
  return sqrt((a.x-b.x)*(a.x-b.x)
             +(a.y-b.y)*(a.y-b.y)
             +(a.z-b.z)*(a.z-b.z));
}

template <typename PointType>
void demo(const int n) {
  pcl::PolygonMesh mesh;

  // add vertices
  pcl::PointCloud<PointType> cloud;
  mesh.cloud.width = cloud.width = n;
  mesh.cloud.height = cloud.height = n;
  //cloud.points.resize(n*n);
  
  for(int j = 0; j < n; j++) {
    for(int i = 0; i < n; i++) {
      PointType point;
      point.x = i; point.y = j; point.z = 0;
      cloud.points.push_back(point);
    }
  }
  pcl::toROSMsg(cloud, mesh.cloud);

  // add triangles in a grid
  for(int j = 0; j < n-1; j++) {
    for(int i = 0; i < n-1; i++) {
      int v = j*n + i;
      pcl::Vertices triangle;
      triangle.vertices.push_back(v);
      triangle.vertices.push_back(v+1);
      triangle.vertices.push_back(v+n+1);
      mesh.polygons.push_back(triangle);
      triangle.vertices.clear();
      triangle.vertices.push_back(v);
      triangle.vertices.push_back(v+n+1);
      triangle.vertices.push_back(v+n);
      mesh.polygons.push_back(triangle);
    }
  }
  
  /// compute (all-points) geodesic
   
  typedef boost::property<boost::edge_weight_t, float> Weight;
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, Weight> Graph;
  Graph graph;

  // construct the graph from the surface
  for(int t = 0; t < mesh.polygons.size(); t++) {
    pcl::Vertices triangle = mesh.polygons[t];
    assert(triangle.vertices.size() == 3);
    int a = triangle.vertices[0], b = triangle.vertices[1], c = triangle.vertices[2];
    // add each edge only once
    if(a < b) add_edge(a, b, Weight(distance(cloud.points[a],cloud.points[b])), graph);
    if(b < c) add_edge(b, c, Weight(distance(cloud.points[b],cloud.points[c])), graph);
    if(c < a) add_edge(c, a, Weight(distance(cloud.points[c],cloud.points[a])), graph);
  }
  
  const size_t E = boost::num_edges(graph), V = boost::num_vertices(graph);
  PCL_INFO ("The graph has %lu vertices and %lu edges.\n", V, E);

  //  allocate N x N destination matrix
  std::vector<std::vector<float> > geodesic_distances;
  geodesic_distances.clear();
  for (size_t i = 0; i < V; ++i) {
    std::vector<float> aux(V);
    geodesic_distances.push_back (aux);
  }
  
  timeval start,end;
  gettimeofday(&start, NULL);
  boost::johnson_all_pairs_shortest_paths(graph, geodesic_distances);
  gettimeofday(&end, NULL);
  
  if(n < 6) {
    for(int i = 0; i < V; i++) {
      for(int j = 0; j < V; j++) {
        if(geodesic_distances[i][j] > n*n) {
          printf("inf      ");
        } else {
          printf("%4f ", geodesic_distances[i][j]);
        }
      }
      printf("\n");
    }
  } else {
    printf("large result, skipping output\n");
  }
  float sec = ((end.tv_sec+end.tv_usec/1e6) - (start.tv_sec+start.tv_usec/1e6));
  printf("computes %i interpoint distances in %f seconds\n", n*n, sec);
}

int main(int argc, char **argv) {
  if(argc < 2) {
    printf("usage: geodesic n\n");
    printf(" computes all-pairs geodesic distances between vertices on an n x n grid\n");
    exit(1);
  }
  demo<pcl::PointNormal>(atoi(argv[1]));
}

