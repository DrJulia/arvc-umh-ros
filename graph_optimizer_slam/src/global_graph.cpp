
#include <graph_optimizer_slam/global_graph.h>

using namespace AISNavigation;
using namespace std;

global_graph::global_graph(){
	restart_map_=true;
	initialized_=false;
}

void global_graph::optimize(){

	/*********************************************************************************
	* optimization
	********************************************************************************/
	// prepare and run the optimization
	ROS_INFO("optimizing...");
	std::string filename = "output";
	cerr << " #nodes:" << pg_.vertices.size() << " #edges:" << pg_.edges.size() << endl; 

	if (!initialized_){
		ROS_INFO("Incremental tree construction... ");
		pg_.buildSimpleTree();
	
		ROS_INFO("Done");
		restart_map_ = false;
		if (restart_map_){
			ROS_INFO("Computing initial guess from observations... ");
			pg_.initializeOnTree();
			ROS_INFO("Done");
		}
	
		ROS_INFO("Initializing the optimizer... ");
		pg_.initializeTreeParameters();
		pg_.initializeOptimization();
		ROS_INFO("Done");

		initialized_=true;
	}
	else{
		pg_.initializeOnlineOptimization();
	}

	double l=pg_.totalPathLength();
	int nEdges=pg_.edges.size();
	double apl=l/(double)(nEdges);
	//ROS_INFO(" Average path length=%f",apl);
	//ROS_INFO(" Complexity of an iteration=%f",l);

	ROS_INFO("**** Starting optimization ****");
	for (int i=0; i<500; i++){
		pg_.iterate();
	}
	double error=pg_.error();
	ROS_INFO(" after last iteration,  global error = %f error/constraint = %f", error, error/nEdges);

//	if (error > 100){
//		ROS_INFO("REBUILDING, ERROR TOO LARGE");
//		rebuild();
//		for (int i=0; i<2000; i++){
//			pg_.iterate();
//			double error=pg_.error();
//				ROS_INFO(" after last iteration,  global error = %f error/constraint = %f", error, error/nEdges);
//		}
//		double error=pg_.error();
//		ROS_INFO(" after last iteration,  global error = %f error/constraint = %f", error, error/nEdges);
//	}
	//Saving optimized graph (gnuplot)... 
	string strippedFilename="/home/mjulia/output";
	string output;
	output=strippedFilename+"-treeopt-optimized.dat";
	pg_.saveGnuplot(output.c_str());

	ROS_INFO("optimization done");
	ROS_INFO("Moving the values to the graph...");
	for (size_t i = 0; i < nodes.size(); i++) {
		nodes[i]->get_optimized_origin().setOrigin( tf::Point(pg_.vertex(i)->pose.x(), pg_.vertex(i)->pose.y(), 0) );
		nodes[i]->get_optimized_origin().setRotation(tf::createQuaternionFromYaw(pg_.vertex(i)->pose.theta()));
	}
}

void global_graph::add_node(graph_node* node){

	nodes.push_back(node);

	pg_.verboseLevel=0;

	// add the node
	ROS_INFO("Add the new node (Total: %d nodes)", nodes.size());
//	ROS_INFO("node %d: (%f,%f) (th: %f)", i,node->get_optimized_origin().getOrigin().getX(), node->get_optimized_origin().getOrigin().getY(), tf::getYaw(node->get_optimized_origin().getRotation()) );
	Operations2D<double>::PoseType p;
//	if (restart_map_){
		p.x() = node->get_optimized_origin().getOrigin().getX();
		p.y() = node->get_optimized_origin().getOrigin().getY();
		p.theta() = tf::getYaw(node->get_optimized_origin().getRotation());
//	}
//	else{
//		p.x() = node->get_optimized_origin().getOrigin().getX();
//		p.y() = node->get_optimized_origin().getOrigin().getY();
//		p.theta() = tf::getYaw(node->get_optimized_origin().getRotation());
//	}
//	ROS_INFO("ADDING NODE (initial): %f, %f, %f,",node->get_initial_origin().getOrigin().getX(), node->get_initial_origin().getOrigin().getY(),tf::getYaw(node->get_initial_origin().getRotation()));
	int nodeid = nodes.size()-1;
	if (!initialized_) pg_.addVertex(nodeid,p);
	ROS_INFO("Node added");

	ROS_INFO("Add the edges (%d new edges)",node->getEdges().size());
	for (std::list<graph_edge>::reverse_iterator it = node->getEdges().rbegin(); it != node->getEdges().rend(); it++){
//	for (std::list<graph_edge>::iterator it = node->getEdges().begin(); it != node->getEdges().end(); it++){

		Operations2D<double>::PoseType p;
		tf::Transform ttt =  it->constraint;
		p.x()=ttt.getOrigin().getX();
		p.y()=ttt.getOrigin().getY();
		p.theta()=tf::getYaw(ttt.getRotation());
		Operations2D<double>::TransformationType t(p);

		//ROS_INFO("found from node %d to node %d: (x: %f, y: %f, th: %f)", nodeid, it->to_node->get_id(), ttt.getOrigin().getX(), ttt.getOrigin().getY(), tf::getYaw(ttt.getRotation()) );

//		ROS_INFO("COV: \n [%e, %e, %e] \n [%e, %e, %e] \n [%e, %e, %e]",
//				 it->constraint_covariance.at<double>(0,0), it->constraint_covariance.at<double>(0,1),it->constraint_covariance.at<double>(0,2),
//				 it->constraint_covariance.at<double>(1,0), it->constraint_covariance.at<double>(1,1),it->constraint_covariance.at<double>(1,2), 
//				 it->constraint_covariance.at<double>(2,0), it->constraint_covariance.at<double>(2,1),it->constraint_covariance.at<double>(2,2));

		Operations2D<double>::InformationType m;
		//Operations2D<double>::CovarianceType c;
//		if (abs(nodeid-it->to_node->get_id()) == 1 || abs(nodeid-it->to_node->get_id()) > 100 ){
//			m.values[0][0] = 1000*it->constraint_covariance.at<double>(0,0);
//			m.values[0][1] = 1000*it->constraint_covariance.at<double>(0,1);
//			m.values[0][2] = 1000*it->constraint_covariance.at<double>(0,2);
//			m.values[1][0] = 1000*it->constraint_covariance.at<double>(1,0);
//			m.values[1][1] = 1000*it->constraint_covariance.at<double>(1,1);
//			m.values[1][2] = 1000*it->constraint_covariance.at<double>(1,2);
//			m.values[2][0] = 1000*it->constraint_covariance.at<double>(2,0);
//			m.values[2][1] = 1000*it->constraint_covariance.at<double>(2,1);
//			m.values[2][2] = 1000*it->constraint_covariance.at<double>(2,2);
//		}
//		else{
			m.values[0][0] = it->constraint_covariance.at<double>(0,0);
			m.values[0][1] = it->constraint_covariance.at<double>(0,1);
			m.values[0][2] = it->constraint_covariance.at<double>(0,2);
			m.values[1][0] = it->constraint_covariance.at<double>(1,0);
			m.values[1][1] = it->constraint_covariance.at<double>(1,1);
			m.values[1][2] = it->constraint_covariance.at<double>(1,2);
			m.values[2][0] = it->constraint_covariance.at<double>(2,0);
			m.values[2][1] = it->constraint_covariance.at<double>(2,1);
			m.values[2][2] = it->constraint_covariance.at<double>(2,2);
//		}
		//m = c.inv();
//		m.values[0][0]=1000.0;  m.values[1][1]=1000.0; m.values[2][2]=100000.0;
//		m.values[0][1]=-1000.0;  m.values[0][2]=1000.0; m.values[1][2]=-10000.0;
//		m.values[1][0]=-1000.0;  m.values[2][0]=1000.0; m.values[2][1]=-10000.0;
		ROS_INFO("adding edge between nodes %d and %d, total nodes: %d", nodeid, it->to_node->get_id(), nodes.size() );
		ROS_INFO("EDGE Information Matrix Diag: %e, %e, %e", m.values[0][0], m.values[1][1], m.values[2][2]);

		float dist1 = sqrt(p.x()*p.x()+p.y()*p.y());
		float dist2 = sqrt(pow(node->get_optimized_origin().getOrigin().getX()-nodes[it->to_node->get_id()]->get_optimized_origin().getOrigin().getX(),2)+
				   pow(node->get_optimized_origin().getOrigin().getY()-nodes[it->to_node->get_id()]->get_optimized_origin().getOrigin().getY(),2));
		//float dist2 = sqrt(pow(pg_.vertex(nodeid)->pose.x()-pg_.vertex(it->to_node->get_id())->pose.x(),2) +
		//		   pow(pg_.vertex(nodeid)->pose.y()-pg_.vertex(it->to_node->get_id())->pose.y(),2));

		//if (fabs(dist1-dist2) < 3){
			//if (abs(nodeid-it->to_node->get_id()) == 1 || abs(nodeid-it->to_node->get_id()) > 100 ){
				if (!initialized_){
					TreePoseGraph2::Vertex* v1=pg_.vertex(nodeid);
					TreePoseGraph2::Vertex* v2=pg_.vertex(it->to_node->get_id());
					pg_.addEdge(v1, v2, t, m);
				}
				else {

					pg_.addIncrementalEdge(nodeid, it->to_node->get_id(), t, m);
					ROS_INFO("done");					
				}
			//}
		//}
//		else{
//			ROS_ERROR("This edge seems wrong, I am going to skip it");
//		}
	}
 	ROS_INFO("All edges added");
};

graph_node::graph_node(){

}



graph_edge::graph_edge(tf::Transform& t, cv::Mat& c, graph_node* to):
	constraint(t),
	constraint_covariance(c),
	to_node(to)
{

}

void global_graph::rebuild(){

	pg_.clear();

	// add the nodes
	ROS_INFO("Add the nodes (%d nodes)",nodes.size());
	for (size_t i = 0; i < nodes.size(); ++i) {
//		ROS_INFO("node %d: (%f,%f) (th: %f)", i,nodes[i]->get_optimized_origin().getOrigin().getX(), nodes[i]->get_optimized_origin().getOrigin().getY(),  tf::getYaw(nodes[i]->get_optimized_origin().getRotation()) );

		Operations2D<double>::PoseType p;
		p.x() = nodes[i]->get_initial_origin().getOrigin().getX();
		p.y() = nodes[i]->get_initial_origin().getOrigin().getY();
		p.theta() = tf::getYaw(nodes[i]->get_initial_origin().getRotation());
		
//		ROS_INFO("ADDING NODE (initial): %f, %f, %f,",nodes[i]->get_initial_origin().getOrigin().getX(), nodes[i]->get_initial_origin().getOrigin().getY(),tf::getYaw(nodes[i]->get_initial_origin().getRotation() ) );
		pg_.addVertex(i,p);
	}
	ROS_INFO("All nodes added");

	ROS_INFO("Add the edges");

	for (size_t i = 0; i < nodes.size(); i++) {
//		ROS_INFO("looking for edges from node %d...", i);
		for (std::list<graph_edge>::reverse_iterator it = nodes[i]->getEdges().rbegin(); it != nodes[i]->getEdges().rend(); it++){
//		for (std::list<graph_edge>::iterator it = nodes[i]->getEdges().begin(); it != nodes[i]->getEdges().end(); it++){

			TreePoseGraph2::Vertex* v1=pg_.vertex(i);
			TreePoseGraph2::Vertex* v2=pg_.vertex(it->to_node->get_id());

			Operations2D<double>::PoseType p;
			tf::Transform ttt =  it->constraint;
			p.x()=ttt.getOrigin().getX();
			p.y()=ttt.getOrigin().getY();
			p.theta()=tf::getYaw(ttt.getRotation());
			Operations2D<double>::TransformationType t(p);

			//ROS_INFO("found from node %d to node %d: (x: %f, y: %f, th: %f)", i, it->to_node->get_id(), ttt.getOrigin().getX(), ttt.getOrigin().getY(), tf::getYaw(ttt.getRotation()) );

//			ROS_INFO("COV: \n [%e, %e, %e] \n [%e, %e, %e] \n [%e, %e, %e]",
//					 it->constraint_covariance.at<double>(0,0), it->constraint_covariance.at<double>(0,1),it->constraint_covariance.at<double>(0,2),
//					 it->constraint_covariance.at<double>(1,0), it->constraint_covariance.at<double>(1,1),it->constraint_covariance.at<double>(1,2), 
//					 it->constraint_covariance.at<double>(2,0), it->constraint_covariance.at<double>(2,1),it->constraint_covariance.at<double>(2,2));

			Operations2D<double>::InformationType m;
			m.values[0][0] = it->constraint_covariance.at<double>(0,0);
			m.values[0][1] = it->constraint_covariance.at<double>(0,1);
			m.values[0][2] = it->constraint_covariance.at<double>(0,2);
			m.values[1][0] = it->constraint_covariance.at<double>(1,0);
			m.values[1][1] = it->constraint_covariance.at<double>(1,1);
			m.values[1][2] = it->constraint_covariance.at<double>(1,2);
			m.values[2][0] = it->constraint_covariance.at<double>(2,0);
			m.values[2][1] = it->constraint_covariance.at<double>(2,1);
			m.values[2][2] = it->constraint_covariance.at<double>(2,2);

			pg_.addEdge(v1, v2, t, m);
		}
	}
 	ROS_INFO("All edges added");

	ROS_INFO("Incremental tree construction... ");
	pg_.buildSimpleTree();

	ROS_INFO("Done");
	restart_map_ = false;
	if (restart_map_){
		ROS_INFO("Computing initial guess from observations... ");
		pg_.initializeOnTree();
		ROS_INFO("Done");
	}

	ROS_INFO("Initializing the optimizer... ");
	pg_.initializeTreeParameters();
	pg_.initializeOptimization();

	initialized_=true;
	ROS_INFO("Done");

}

/////////////////
//COPY SEG OF OLD OPTIMIZATION NOT INCREMENTAL
/////////////////


//void global_graph::optimize(){

//	/*********************************************************************************
//	* creating the optimization problem
//	********************************************************************************/
//	TreeOptimizer2 pg;
//	pg.verboseLevel=0;

//	// add the nodes
//	ROS_INFO("Add the nodes (%d nodes)",nodes.size());
//	for (size_t i = 0; i < nodes.size(); ++i) {
////		ROS_INFO("node %d: (%f,%f) (th: %f)", i,nodes[i]->get_optimized_origin().getOrigin().getX(), nodes[i]->get_optimized_origin().getOrigin().getY(),  tf::getYaw(nodes[i]->get_optimized_origin().getRotation()) );

//		Operations2D<double>::PoseType p;
//		if (restart_map_){
//			p.x() = nodes[i]->get_initial_origin().getOrigin().getX();
//			p.y() = nodes[i]->get_initial_origin().getOrigin().getY();
//			p.theta() = tf::getYaw(nodes[i]->get_initial_origin().getRotation());
//		}
//		else{
//			p.x() = nodes[i]->get_optimized_origin().getOrigin().getX();
//			p.y() = nodes[i]->get_optimized_origin().getOrigin().getY();
//			p.theta() = tf::getYaw(nodes[i]->get_optimized_origin().getRotation());
//		}
////		ROS_INFO("ADDING NODE (initial): %f, %f, %f,",nodes[i]->get_initial_origin().getOrigin().getX(), nodes[i]->get_initial_origin().getOrigin().getY(),tf::getYaw(nodes[i]->get_initial_origin().getRotation() ) );
//		pg.addVertex(i,p);
//	}
//	ROS_INFO("All nodes added");

//	ROS_INFO("Add the edges");

//	for (size_t i = 0; i < nodes.size(); i++) {
////		ROS_INFO("looking for edges from node %d...", i);
//		for (std::list<graph_edge>::reverse_iterator it = nodes[i]->getEdges().rbegin(); it != nodes[i]->getEdges().rend(); it++){
////		for (std::list<graph_edge>::iterator it = nodes[i]->getEdges().begin(); it != nodes[i]->getEdges().end(); it++){

//			TreePoseGraph2::Vertex* v1=pg.vertex(i);
//			TreePoseGraph2::Vertex* v2=pg.vertex(it->to_node->get_id());

//			Operations2D<double>::PoseType p;
//			tf::Transform ttt =  it->constraint;
//			p.x()=ttt.getOrigin().getX();
//			p.y()=ttt.getOrigin().getY();
//			p.theta()=tf::getYaw(ttt.getRotation());
//			Operations2D<double>::TransformationType t(p);

//			//ROS_INFO("found from node %d to node %d: (x: %f, y: %f, th: %f)", i, it->to_node->get_id(), ttt.getOrigin().getX(), ttt.getOrigin().getY(), tf::getYaw(ttt.getRotation()) );

////			ROS_INFO("COV: \n [%e, %e, %e] \n [%e, %e, %e] \n [%e, %e, %e]",
////					 it->constraint_covariance.at<double>(0,0), it->constraint_covariance.at<double>(0,1),it->constraint_covariance.at<double>(0,2),
////					 it->constraint_covariance.at<double>(1,0), it->constraint_covariance.at<double>(1,1),it->constraint_covariance.at<double>(1,2), 
////					 it->constraint_covariance.at<double>(2,0), it->constraint_covariance.at<double>(2,1),it->constraint_covariance.at<double>(2,2));

//			Operations2D<double>::InformationType m;
////			Operations2D<double>::CovarianceType c; 
////			c.values[0][0] = it->constraint_covariance.at<float>(0,0);
////			c.values[0][1] = it->constraint_covariance.at<float>(0,1);
////			c.values[0][2] = it->constraint_covariance.at<float>(0,2);
////			c.values[1][0] = it->constraint_covariance.at<float>(1,0);
////			c.values[1][1] = it->constraint_covariance.at<float>(1,1);
////			c.values[1][2] = it->constraint_covariance.at<float>(1,2);
////			c.values[2][0] = it->constraint_covariance.at<float>(2,0);
////			c.values[2][1] = it->constraint_covariance.at<float>(2,1);
////			c.values[2][2] = it->constraint_covariance.at<float>(2,2);
//			//m = c.inv();

//			m.values[0][0]=1.0;  m.values[1][1]=1.0; m.values[2][2]=1.0;
//			m.values[0][1]=0;  m.values[0][2]=0; m.values[1][2]=0;

//			float dist1 = sqrt(p.x()*p.x()+p.y()*p.y());
//			//float dist2 = sqrt(pow(nodes[i]->get_initial_origin().getOrigin().getX()-nodes[it->to_node->get_id()]->get_initial_origin().getOrigin().getX(),2)+
//			//		   pow(nodes[i]->get_initial_origin().getOrigin().getY()-nodes[it->to_node->get_id()]->get_initial_origin().getOrigin().getY(),2));
//			float dist2 = sqrt(pow(pg.vertex(i)->pose.x()-pg.vertex(it->to_node->get_id())->pose.x(),2) +
//					   pow(pg.vertex(i)->pose.y()-pg.vertex(it->to_node->get_id())->pose.y(),2));
//			if (fabs(dist1-dist2) < 2.5){
//				pg.addEdge(v1, v2, t, m);
//			}
////			else{
////				ROS_ERROR("This edge seems wrong, I am going to skip it");
////			}
//		}
//	}
// 	ROS_INFO("All edges added");

//	/*********************************************************************************
//	* optimization
//	********************************************************************************/

//	// prepare and run the optimization
//	ROS_INFO("optimizing...");

//	std::string filename = "output";

//	cerr << " #nodes:" << pg.vertices.size() << " #edges:" << pg.edges.size() << endl; 

//	if (false){
//		cerr << "Loading equivalence constraints and collapsing nodes... ";
//		pg.loadEquivalences(filename.c_str());
//		cerr << "Done" << endl; 
//		cerr << " #nodes:" << pg.vertices.size() << " #edges:" << pg.edges.size() << endl; 
//	}

//	if (false){
//		cerr << "Compressing indices... ";
//		pg.compressIndices();
//		cerr << "Done" << endl;
//	}

//	switch (0){
//		case 0:
//			cerr << "Incremental tree construction... ";
//			pg.buildSimpleTree();
//			cerr << "Done" << endl;
//			break;
//		case 1:
//			cerr << "MST construction... ";
//			pg.buildMST(pg.vertices.begin()->first);
//			cerr << "Done" << endl;
//			break;
//		default:
//			cerr << " FATAL ERROR: Invalid tree type. Aborting!";
//			return ;
//	}

//	if (restart_map_){
//		cerr << "Computing initial guess from observations... ";
//		pg.initializeOnTree();
//		cerr << "Done" << endl;
//	}

//	cerr << "Initializing the optimizer... ";
//	pg.initializeTreeParameters();
//	pg.initializeOptimization();
//	double l=pg.totalPathLength();
//	int nEdges=pg.edges.size();
//	double apl=l/(double)(nEdges);
//	cerr << "Done" << endl;

//	cerr << " Average path length=" << apl << endl;
//	cerr << " Complexity of an iteration=" << l  << endl;

//	string strippedFilename="/home/mjulia/output";
//	string extension="ext";

//	string output;
////	cerr << "Saving starting graph... ";
////	output=strippedFilename+"-treeopt-initial.graph";
////	pg.save(output.c_str());
////	cerr << "Done" << endl << endl;

////	cerr << "Saving starting graph (gnuplot)... ";
////	output=strippedFilename+"-treeopt-initial.dat";
////	pg.saveGnuplot(output.c_str());
////	cerr << "Done" << endl << endl;

//	cerr << "**** Starting optimization ****" << endl;
//	for (int i=0; i<500; i++){
//		pg.iterate();
//		if (false){
//			char b[10];
//			sprintf(b,"%04d",i);
//			//string output=strippedFilename+"-treeopt-" + b + ".dat";
//			//pg.saveGnuplot(output.c_str());
//		}
//		if (false){
//			// compute the error and dump it
//			double error=pg.error();
//			cerr << "iteration = " << i << "  global error = " << error << "   error/constraint = " << error/nEdges << endl;
//		}
//	}
//	double error=pg.error();
//	cerr << " after last iteration,  global error = " << error << "   error/constraint = " << error/nEdges << endl;
////	
////	if (error > 0.5 ){ 
////		for (int i=0; i<500; i++) pg.iterate();	// 500 times more
////		error=pg.error(); 
////		cerr << " extra iterations,  global error = " << error << "   error/constraint = " << error/nEdges << endl;
////		if (error > 0.5 ) restart_map_ = true;
////	}
////	else restart_map_ = false;
//	restart_map_ = false;

//	//cerr << "Saving optimized graph (gnuplot)... ";
//	output=strippedFilename+"-treeopt-optimized.dat";
//	pg.saveGnuplot(output.c_str());
//	//cerr << "Done" << endl << endl;

//	ROS_INFO("optimization done");

//	ROS_INFO("Moving the values to the graph...");

//	for (size_t i = 0; i < nodes.size(); i++) {
//		nodes[i]->get_optimized_origin().setOrigin( tf::Point(pg.vertex(i)->pose.x(), pg.vertex(i)->pose.y(), 0) );
//		nodes[i]->get_optimized_origin().setRotation(tf::createQuaternionFromYaw(pg.vertex(i)->pose.theta()));
//	}
//}



