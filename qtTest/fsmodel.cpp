#include "fsmodel.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <boost/filesystem.hpp>

using namespace std;
using namespace pcl;

FSModel::FSModel()
{
    pointCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

void FSModel::convertPointCloudToSurfaceMesh()
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    //sensor_msgs::PointCloud2 cloud_blob;
    //pcl::io::loadPCDFile ("bearHigh.pcd", cloud_blob);
    //pcl::fromROSMsg (cloud_blob, *cloud);
    //* the data should be available in cloud

    cloud->points.resize(pointCloud->size());
    for (size_t i = 0; i < pointCloud->points.size(); i++) {
        cloud->points[i].x = pointCloud->points[i].x;
        cloud->points[i].y = pointCloud->points[i].y;
        cloud->points[i].z = pointCloud->points[i].z;
    }

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setRadiusSearch(15.0);
    //n.setKSearch (30);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (15.00);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (surfaceMesh);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    //pcl::io::savePLYFile("mesh.ply", surfaceMesh);
    FSController::getInstance()->meshComputed=true;
}

void FSModel::loadSurfaceMeshFromOFF(const std::string &offFilePath)
{
    ifstream offFile; //the file object (read only)
    offFile.open(offFilePath.c_str(),ios::in); //open file for input
    cout << offFilePath.c_str() << endl;
    if(!offFile.is_open()){
        cout << "Could not open file! " << offFilePath.c_str() <<  endl;
        return;
    }

    /* read first line and checks wether it is correct or not according to .off format */
    string fileFormat;
    offFile >> fileFormat; //first line/word of a .off should be OFF
    cout << fileFormat << endl;
    //printf("%s filePath %s \n",__PRETTY_FUNCTION__,offFilePath);
    if(fileFormat.compare("OFF") != 0){ //if first line is not OFF
      cout << "provided file is not a valid .off file!" << endl;
      return;
    }else{
        cout << "Valid .off file!" << endl;
    }

    /* clear old loaded model before opening new one */
    surfaceMesh.cloud.data.clear();
    surfaceMesh.polygons.clear();

    //vertexVector.clear();
    //faceVector.clear();

    /* read 2. line, containg number of vertices, faces and edges */
    unsigned int numberOfVertices;  //needed to determine size of vector
    unsigned int numberOfFaces;     //needed to determine size of vector
    unsigned int numberOfEdges; //not needed

    offFile >> numberOfVertices;
    offFile >> numberOfFaces;
    offFile >> numberOfEdges;

    PointCloud<PointXYZ> cloud;
    //cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud.points.resize(numberOfVertices);

    /* reserver space for vertexVector and faceVector according to numberOfVertices and numberOfEdges */
    //vertexVector.reserve(numberOfVertices);
    //faceVector.reserve(numberOfFaces)

    /* read vertices=tripple of points from .off file and store them in vertexVector */
    for(unsigned int i=0;i<numberOfVertices;i++){
      while(offFile.peek()=='#') offFile.ignore(256,'\n');
      offFile >> cloud.points[i].x;
      cout << cloud.points[i].x << endl;
      while(offFile.peek()=='#') offFile.ignore(256,'\n');
      offFile >> cloud.points[i].y;
      while(offFile.peek()=='#') offFile.ignore(256,'\n');
      offFile >> cloud.points[i].z;
      offFile.ignore(256,'\n'); //ignore rest of line
    }


    //pcl::toROSMsg(cloud, surfaceMesh.cloud);//not a member since pcl 1.7
    pcl::toPCLPointCloud2(cloud, surfaceMesh.cloud);//for pcl >= 1.7

    /* read faces from .off file and store them in faceVector */
    surfaceMesh.polygons.reserve(numberOfFaces);
    for(unsigned int i=0;i<numberOfFaces;i++){
      unsigned int n; //number of vertices on this face
      offFile >> n;
      surfaceMesh.polygons[i].vertices.reserve(n);
      for(unsigned int j=0;j<n;j++){
        while(offFile.peek()=='#') offFile.ignore(256,'\n');
        offFile >> surfaceMesh.polygons[i].vertices[j];
      }
      offFile.ignore(256,'\n'); //ignore rest of line
    }
    offFile.close();
    return;
}

void FSModel::convertPointCloudToSurfaceMesh2()
{
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPoisson;
    //pcl::PolygonMesh surfaceMesh;
    //pcl::PolygonMesh surfaceMeshPoisson;
    boost::filesystem::path p;
    p = boost::filesystem::current_path();

    //this is platform specific code, needs to be changed for other platforms then mac!
    p/="powercrust";
    const char* resPath;
    if(boost::filesystem::exists(p)){
        //powercrust is in same directory as fabscan executable
        cout << "file found!" << endl;
        resPath = "./";
    }else{
        //not in the same directory,
        //here we assume we are on mac and this need to go inside the .app bundle
        cout << "file not found" << endl;
        resPath = "./FabScan100.app/Contents/MacOS/";
    }

    int sysRet;
    std::string ptsFilePath;
    ptsFilePath.append(resPath);
    ptsFilePath.append("pc.pts");
    this->savePointCloudAsPTS(ptsFilePath);
    char* command;

    asprintf(&command,"cd %s; ./powercrust -i %s -R 1.5 -B -m 10000", resPath, "pc.pts");
    sysRet = system(command);
    cerr << command << " system: " << sysRet << endl;

    if(sysRet==0){
        FSController::getInstance()->meshComputed=true;
    }
    asprintf(&command,"cd %s; ./orient -i pc.off -o final.off",resPath);
    sysRet = system(command);
    cerr << command << " system: " << sysRet << endl;

    char* offFilePath;
    asprintf(&offFilePath,"%sfinal.off",resPath);
    cerr << "loading surface from off..." << endl;
    this->openFromOFFFile(offFilePath);

    /*this->loadSurfaceMeshFromOFF(offFilePath);
    cout << "loaded surface from off!" << endl;
    if(sysRet==0){
        FSController::getInstance()->meshComputed=true;
    }
    cout << "loading off file works complete!" << endl;
    FSController::getInstance()->meshComputed=true;
    return;*/
}

void FSModel::convertPointCloudToSurfaceMesh3()
{
    /*pcl::MovingLeastSquares<PointXYZRGB, PointXYZ> mls;
    mls.setInputCloud (pointCloud);
    mls.setSearchRadius (0.01);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (2);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<PointXYZRGB, PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius (0.005);
    mls.setUpsamplingStepSize (0.003);

    pcl::PointCloud<PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<PointXYZ> ());
    mls.process (*cloud_smoothed);*/

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize(pointCloud->size());
    for (size_t i = 0; i < pointCloud->points.size(); i++) {
        cloud->points[i].x = pointCloud->points[i].x;
        cloud->points[i].y = pointCloud->points[i].y;
        cloud->points[i].z = pointCloud->points[i].z;
    }

    pcl::NormalEstimation<pcl::PointXYZ, Normal> ne;
    //ne.setNumberOfThreads(8);
    ne.setInputCloud (cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (0.01);
    Eigen::Vector4f centroid;
    compute3DCentroid (*cloud, centroid);
    ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
    qDebug() << "Centroid:"<<centroid[0] <<centroid[1]<<centroid[2];
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());
    ne.compute (*cloud_normals);
    //invert all normals, primarly they all point to the centroid
    for (size_t i = 0; i < cloud_normals->size (); ++i) {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }
    PointCloud<PointNormal>::Ptr cloud_smoothed_normals (new PointCloud<PointNormal> ());
    concatenateFields (*cloud, *cloud_normals, *cloud_smoothed_normals);
    //concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);*/

    Poisson<PointNormal> poisson;
    poisson.setScale(1.0);
    poisson.setDepth (9);
    poisson.setDegree(2);
    poisson.setSamplesPerNode(3);
    poisson.setIsoDivide(8);
    poisson.setConfidence(0);
    poisson.setManifold(0);
    poisson.setOutputPolygons(0);
    poisson.setSolverDivide(8);
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.reconstruct(surfaceMeshPoisson);
    //pcl::io::savePLYFile("meshPoisson.ply", surfaceMeshPoisson);
    FSController::getInstance()->meshComputed=true;
}

void FSModel::loadPointCloudFromPCD(const std::string &file_name)
{
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file_name, *pointCloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read pcd file  \n");
        return ;
    }
    std::cout   << "Loaded "
                << pointCloud->width * pointCloud->height
                << " data points from: "<< file_name
                << std::endl;
    FSController::getInstance()->geometries->setPointCloudTo(pointCloud);
    std::cout << "Done from Model" << std::endl;
}

void FSModel::loadPointCloudFromPLY(const std::string &file_name)
{
    if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (file_name, *pointCloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read ply file  \n");
        return ;
    }
    std::cout   << "Loaded "
                << pointCloud->width * pointCloud->height
                << " data points from: "<< file_name
                << std::endl;
    FSController::getInstance()->geometries->setPointCloudTo(pointCloud);
    std::cout << "Done from Model" << std::endl;
}

void FSModel::savePointCloudAsPCD(const std::string &file_name)
{
    if(pointCloud->size()==0) return;
    pcl::io::savePCDFileASCII (file_name, *pointCloud);
    std::cerr << "Saved " << pointCloud->points.size() << " data points to " << file_name << std::endl;
}

void FSModel::savePointCloudAsPLY(const std::string &file_name)
{
    if(pointCloud->size()==0) return;
    pcl::io::savePLYFileASCII (file_name, *pointCloud);
    std::cerr << "Saved " << pointCloud->points.size() << " data points to " << file_name << std::endl;
}

void FSModel::savePointCloudAsPTS(const std::string &file_name)
{
    if(pointCloud->size()==0) return;
    ofstream ptsFile;
    string ptsFilename;
    ptsFilename.assign(file_name);
    ptsFile.open (ptsFilename.c_str());

    for (size_t i = 0; i < pointCloud->points.size(); ++i){
        ptsFile << pointCloud->points[i].x << " " << pointCloud->points[i].y << " " << pointCloud->points[i].z << endl;
    }

    ptsFile.close();

    std::cerr << "Saved " << pointCloud->points.size() << " data points to " << file_name << std::endl;
    return;
}

void FSModel::addPointToPointCloud(FSPoint point)
{
    //qDebug()<<"added Point to cloud";
    pcl::PointXYZRGB p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    int rgb = ((int)point.color.red) << 16 | ((int)point.color.green) << 8 | ((int)point.color.blue);
    p.rgb = rgb;
    pointCloud->push_back(p);
}

unsigned int FSModel::openFromOFFFile(const char* offFilePath){
  cout << "Open from off file: " << offFilePath << endl;
  ifstream offFile; //the file object (read only)

  offFile.open(offFilePath,ios::in); //open file for input

  /* read first line and checks wether it is correct or not according to .off format */
  string fileFormat;
  offFile >> fileFormat; //first line/word of a .off should be OFF
  printf("%s filePath %s \n", __PRETTY_FUNCTION__, offFilePath);
  if(fileFormat.compare("OFF") != 0){ //if first line is not OFF
    printf("provided file is not a valid .off file \n");
    return 1;
  }

  /* clear old loaded model before opening new one */
  vertexVector.clear();
  faceVector.clear();

  /* read 2. line, containg number of vertices, faces and edges */
  unsigned int numberOfVertices;  //needed to determine size of vector
  unsigned int numberOfFaces;     //needed to determine size of vector
  unsigned int numberOfEdges; //not needed

  offFile >> numberOfVertices;
  offFile >> numberOfFaces;
  offFile >> numberOfEdges;

  /* reserver space for vertexVector and faceVector according to numberOfVertices and numberOfEdges */
  vertexVector.reserve(numberOfVertices);
  faceVector.reserve(numberOfFaces);

  /* read vertices=tripple of points from .off file and store them in vertexVector */
  for(int i=0;i<numberOfVertices;i++){
    FSFloat x,y,z;
    while(offFile.peek()=='#') offFile.ignore(256,'\n');
    offFile >> x;
    while(offFile.peek()=='#') offFile.ignore(256,'\n');
    offFile >> y;
    while(offFile.peek()=='#') offFile.ignore(256,'\n');
    offFile >> z;
    offFile.ignore(256,'\n'); //ignore rest of line
    FSPoint point = FSMakePoint(x,y,z); //TODO: delete points in decronstructor
    vertexVector.push_back(point);
  }

  /* read faces from .off file and store them in faceVector */
  for(int i=0;i<numberOfFaces;i++){
    unsigned int n; //number of vertices on this face
    offFile >> n;
    vector<unsigned int> indiceVector;
    indiceVector.reserve(n);
    for(int j=0;j<n;j++){
      unsigned int z; //indice of vertex
      while(offFile.peek()=='#') offFile.ignore(256,'\n');
      offFile >> z;
      indiceVector.push_back(z); //add this vertex to vector of vertices per face
    }
    offFile.ignore(256,'\n'); //ignore rest of line
    faceVector.push_back(indiceVector);
  }
  //if(fileFormat !=)
  //if(fileFormat != "OFF"){ cout << "error wrong file format" << endl; return 1; }
  offFile.close();
  cout << "done reading off file" << endl;
  return 0;
}

unsigned int FSModel::convertPolygons2Triangles(void){
    cout << "convertPolygons2Triangles" << endl;
  vector<vector <unsigned int> > newFaceVector; //the new vector which stores the triangles instead of polygons

  /* find out number of triangles and reserve space for them */
  unsigned int numberOfNewFaces = 0;
  for(int i=0;i<faceVector.size();i++){ //iterate throught the polygons
    vector <unsigned int> polygon = faceVector[i];
    numberOfNewFaces += polygon.size()-2; //minus 2 'cos triangle has 3 vertices = 1 face
  }
  newFaceVector.reserve(numberOfNewFaces); //reserve space

  //cout << "#old: " << faceVector.size() << " #new: " << numberOfNewFaces << endl;

  /* decompose polygons into triangles */
  for(int i=0;i<faceVector.size();i++){ //iterate throught the polygons
    vector <unsigned int> polygon = faceVector[i];

    while(polygon.size() > 3){ //current polygon is not a triangle, so further inspection is needed
      vector <unsigned int> triangle(3,1);
      triangle.reserve(3);
      triangle[0]=polygon[0];
      triangle[2]=polygon.back();
      polygon.pop_back(); //remove last element
      triangle[1]=polygon.back();
      newFaceVector.push_back(triangle);
    }
    newFaceVector.push_back(polygon);
  }

  faceVector = newFaceVector; //assign new vector
  cout << "convertPolygons2Triangles done" << endl;

  return 0;
}

unsigned int FSModel::saveToSTLFile(string stlFilePath){
    cout << "saveToSTLFile" << endl;
    cout << "writing file: " << stlFilePath << endl;
  ofstream stlFile;
  stlFile.open(stlFilePath.c_str() ,ios::out);
  stlFile << "solid 3dscan" << endl;

  for(int i=0;i<faceVector.size();i++){

    FSPoint p1 = vertexVector[ faceVector[i][0] ];
    FSPoint p2 = vertexVector[ faceVector[i][1] ];
    FSPoint p3 = vertexVector[ faceVector[i][2] ];

    //generate the 3 vectors from the polygon
    FSPoint v1,v2,n;
    v1.x=p1.x-p2.x;
    v1.y=p1.y-p2.y;
    v1.z=p1.z-p2.z;

    v2.x=p2.x-p3.x;
    v2.y=p2.y-p3.y;
    v2.z=p2.z-p3.z;

    n.x = v1.y*v2.z - v1.z*v2.y;
    n.y = v1.z*v2.x - v1.x*v2.z;
    n.z = v1.x*v2.y - v1.y*v2.x;

    FSFloat d = sqrtf(n.x*n.x+n.y*n.y+n.z*n.z);

    n.x /= d;
    n.y /= d;
    n.z /= d;

    stlFile << "facet normal " << n.x  << " " << n.y << " " << n.z << endl;
    stlFile << "  outer loop" << endl;
    for(int j=0;j<faceVector[i].size();j++){
      unsigned int indice = faceVector[i][j];
      FSPoint p = vertexVector[indice];
      //axes are permuted to comply with 3d printing standart where the x-y plane is horizontal
        stlFile << "    vertex ";
        //stlFile << p.y*10 << " ";
        //stlFile << p.z*10+70 << " "; //all vertex coordinates must be positive (stl) so add radius of turntable
        //stlFile << p.x*10+70 << " ";
        stlFile << p.x << " ";
        stlFile << p.y << " "; //all vertex coordinates must be positive (stl) so add radius of turntable
        stlFile << p.z << " ";
        stlFile << endl;
    }
    stlFile << "  endloop" << endl;
    stlFile << "endfacet" << endl;
  }
  stlFile << "endsolid test" << endl;
  stlFile.close();
  return 0;
}
