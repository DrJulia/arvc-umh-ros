	
#include <ros/ros.h>
#include <explore_tree_planner/treeNode.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <opencv/highgui.h>

treeNode::treeNode(int type):
	parent_(0),
	node_type_(type),
	area_(0),
	localCost_(0.0f),
	globalCost_(0.0f),
	areaValue_(0.0f),
	cumulatedAreaValue_(0.0f),
	profit_(0.0f)
{

}

treeNode::~treeNode(){
	if (area_) delete area_;
	for (uint n=0; n<children_.size(); n++){
		delete children_[n];
	}
}

void treeNode::addChildren(treeNode& child){
	child.setParent(*this);
	children_.push_back(&child);
}

treeNode& treeNode::getChildren(int number) const {
	assert(number < (int)children_.size());
	return *(children_[number]);
}

treeNode& treeNode::getParent() const{
	assert(parent_);
	return (*parent_);
}

void treeNode::eraseChild(treeNode* t){
	for (int i =0; i < children_.size(); i++){
		if (children_[i] == t) {
			children_.erase(children_.begin()+i);
			break;
		}
	}
}

bool treeNode::orderByValue(const treeNode* a, const treeNode* b){
	if (a == b) return false;
	return (((treeNode*)a)->getCumulatedAreaValue() > ((treeNode*)b)->getCumulatedAreaValue());
}

bool treeNode::orderByCost(const treeNode* a, const treeNode* b){
	if (a == b) return false;
	else return (((treeNode*)a)->getGlobalCost() > ((treeNode*)b)->getGlobalCost());
}

bool treeNode::orderByProfit(const treeNode* a, const treeNode* b){
	if (a == b) return false;
	else return (((treeNode*)a)->getProfit() > ((treeNode*)b)->getProfit());
}

treeNode* treeNode::createTreePartitioning(const gridMapInterface& omap, const geometry_msgs::Point& rcell, int dilationradius, int actionradius, int sensorradius, int min_cluster_size){

	// we create the root node
	treeNode* root_node = new treeNode(treeNodeTypes::GATEWAY_NODE);
	root_node->setCell(rcell);

	// omap.showMap("omap");
//	IplImage* img = omap.getMapAsImage();
//	cvCircle(img, cvPoint(rcell.x, omap.getHeight()-rcell.y), 10, cvScalar(255,0,0) );
//	cvNamedWindow("omap", 0);
//	cvShowImage("omap",img);
//	cvReleaseImage(&img);

	// and a white processed area
	binMap processed(omap.getWidth(), omap.getHeight());

	// this is the open list
	std::vector<treeNode*> node_open_list;

	// we add the root node to the open list
	node_open_list.push_back(root_node);

//	binMap currentSZ;
//	omap.sz(root_node->getX(), root_node->getY(), currentSZ, dilationradius, actionradius);				// get its coverage area
//	omap.secview(root_node->getX(), root_node->getY(), currentSZ, dilationradius, actionradius);				// get its coverage area
	//currentSZ.showMap("sz");
	//cv::waitKey(25);
	int count=1;

	ros::WallTime initPlanTime = ros::WallTime::now();	
	// and begin processing
	while (node_open_list.size()>0){
		std::sort(node_open_list.begin(), node_open_list.end(), treeNode::orderByCost);					// sort the list by global cost
		treeNode* node = node_open_list.back();										// extract the node with smallest cost
		node_open_list.pop_back();
		ros::WallTime endPlanTime = ros::WallTime::now();	
		double totalTime = endPlanTime.toSec() - initPlanTime.toSec();		
		//ROS_INFO("Nodes on open list %d, elapsed time %f",node_open_list.size(), totalTime);

		if (!node->isLeaf()){ // GATEWAY nodes
//			ROS_INFO("proceesing gateway...");
			binMap coverageArea;
//			ROS_INFO("get safe zone...");	
//			omap.sz(node->getX(), node->getY(), coverageArea, dilationradius, actionradius);			// get its coverage area
			omap.secview(node->getX(), node->getY(), coverageArea, dilationradius, actionradius);			// get its coverage area
			int remaining = node_open_list.size();
			for (int i =0; i < remaining; i++){
				if ( coverageArea.get(node_open_list[i]->getX(),  node_open_list[i]->getY()) &&
					node_open_list[i]->getGlobalCost() > ( node->getGlobalCost() +
					sqrt(pow(node_open_list[i]->getX()-node->getX(),2)+pow(node_open_list[i]->getY()-node->getY(),2)))  ){
					node_open_list[i]->getParent().eraseChild(node_open_list[i]);
					node_open_list.erase (node_open_list.begin()+i);
					remaining--;
				}
			}
//			ROS_INFO("creating filter...");
			binMap* filter = new binMap(coverageArea);								// copy it to the filter	
//			ROS_INFO("applying mask...");
			filter->sub(processed);											// remove processed parts of the coverage area
//			ROS_INFO("remove unconnected...");
			//filter->removeUnconnected(node->getX(), node->getY());							// remove unconnected components
			filter->set(node->getX(), node->getY(), true);
			node->setArea(*filter);											// what remains is the area associated to the node
//			ROS_INFO("seek child nodes...");
			seekChildNodes(omap, *node, coverageArea, *filter, min_cluster_size);					// seek for gateways in that area
//			ROS_INFO("child nodes search finish");
			for(int i = 0; i< node->getNChildren(); i++){
				node_open_list.push_back(&node->getChildren(i));						// add nodes to the open list with the items found
				count++;		
			}
//			ROS_INFO("add to mask...");
			processed.add(*filter);											// we add the new area to the processed zones 
//			ROS_INFO("added");
			//delete filter;
		}
		else{		// leaf nodes
//			ROS_INFO("node is a leaf");
//			binMap* coverageArea = new binMap();
////			ROS_INFO("get expected safe zone...");			
//			omap.fov(node->getX(), node->getY(), *coverageArea, 0, sensorradius);			// get its coverage area
////			ROS_INFO("applying mask...");
//			coverageArea->sub(processed);											// remove processed parts of the coverage area
////			ROS_INFO("remove unconnected...");
//			coverageArea->removeUnconnected(node->getX(), node->getY());							// remove unconnected components
////			ROS_INFO("done");
//			coverageArea->set(node->getX(), node->getY(), true);
//			node->setArea(*coverageArea);

		}
	};
	//processed.showMap("processed");
	//cv::waitKey(10);

	ROS_INFO("Tree created with %d nodes", count);
	return root_node;
}

void treeNode::seekChildNodes(const gridMapInterface& omap, treeNode& node, binMap& area, binMap& filter, int min_cluster_size){
	
	// find gateways nodes
	binMap gateways;
	omap.gateways(area, gateways);
	// remove the gateways that go back to previous analysed parts of the environment, we create a tree with no loops and no overlaping
	gateways.times(filter);
	// cluster contiguous gateways cells and add a node to the tree for each cluster
	cluster(gateways, node, treeNodeTypes::GATEWAY_NODE, min_cluster_size); 

	binMap frontiers;
	int nfront = omap.frontiersIn(filter, frontiers);
	if (nfront>0) cluster(frontiers, node, treeNodeTypes::LEAF_TYPE_FRONTIER, min_cluster_size); // añadir los nodos frontera
}

int treeNode::cluster(const binMap& map, treeNode& parent, int nodetype, int min_cluster_size){

	int count = 0;	// number of clusters found, the method returns this value
	binMap aux(map.getWidth(), map.getHeight(), map.getResolution(), map.getXOrigin(), map.getYOrigin());
	geometry_msgs::Point p;

	for (int i = map.getRoi().x ; i < map.getRoi().x + map.getRoi().width ; i++){
		for (int j = map.getRoi().y ; j < map.getRoi().y + map.getRoi().height ; j++){  			// for each cell

			std::list<point> seq;
			clustering(i,j,aux,map,seq,true);
			if ((int)seq.size() >= min_cluster_size ){ 	

				std::list<point>::iterator it;								// find the cell in the middle of the cluster
				uint k;
				
//				int numsamples = 1;
				int numsamples = seq.size()/100 + 1;
				for (int s = 0; s < numsamples; s++){
					for( k = 0, it = seq.begin(); it != seq.end() && k <= (s+0.5)*seq.size()/numsamples; it++ , k++);		
					it--; k--;
					p.x = it->x; p.y = it->y;

					treeNode* newNode = new treeNode(nodetype); 						// new node
					newNode->setCell(p);									// set the position to the middle cell
					newNode->setSize(seq.size());
					float d = EPS+sqrt(pow(((float)parent.getX())-p.x,2)+pow(((float)parent.getY())-p.y,2));// calculate the cost as the euclidean distance
					newNode->setLocalCost(d); 								// set local cost
					newNode->setGlobalCost(parent.getGlobalCost() + d); 					// set global cost
					parent.addChildren(*newNode);								// add the node to the tree

					count++;										
				}
			}
		}
	}

	return count;
}

bool treeNode::clustering(const int &i, const int &j, binMap& aux, const binMap& map, std::list<point> &seq, bool backfront){

	if(aux.get(i,j)==false && map.get(i,j) == true ){
		point p(i,j);
		if (backfront)	seq.push_back(p);
		else		seq.push_front(p);
		aux.set(i,j,true);
		int x, y;
		for (x = i-1; x<=i+1; x++)
			for (y = j-1; y<=j+1; y++)
				if (clustering(x,y,aux,map,seq,backfront))
					backfront = !backfront;
		return true; 
	}
	else{
		aux.set(i,j,true);
		return false; 
	}
}

const treeNode* treeNode::findMaxProfitNode(double& bestProfit){
	const treeNode *bestNode = this;
	bestProfit = getProfit();
	double profit;
	for (uint n=0; n<children_.size(); n++){
		const treeNode* node = children_[n]->findMaxProfitNode(profit);
		if (profit > bestProfit){
			bestProfit = profit;
			bestNode = node;
		}
	}
	return bestNode;
}
