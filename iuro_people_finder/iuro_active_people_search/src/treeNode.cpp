	
	
#include <iuro_active_people_search/treeNode.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <opencv/highgui.h>

treeNode::treeNode(int type):
	parent_(0),
	node_type_(type),
	localCost_(0.0f),
	globalCost_(0.0f),
	areaValue_(0.0f),
	cumulatedAreaValue_(0.0f),
	profit_(0.0f)
{

}

treeNode::~treeNode(){
	delete area_;
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

treeNode* treeNode::createTreePartitioning(const gridMapInterface& omap, const geometry_msgs::Point& rcell, int dilationradius, int actionradius, int min_cluster_size){

	// we create the root node
	treeNode* root_node = new treeNode(treeNodeTypes::GATEWAY_NODE);
	root_node->setCell(rcell);

	// and a white processed area
	binMap processed(omap.getWidth(), omap.getHeight());

	// this is the open list
	std::vector<treeNode*> node_open_list;

	// we add the root node to the open list
	node_open_list.push_back(root_node);

	int count=1;

	// and begin processing
	while (node_open_list.size()>0){
		std::sort(node_open_list.begin(), node_open_list.end(), treeNode::orderByCost);				// sort the list by global cost
		treeNode* node = node_open_list.back();									// extract the node with smallest cost
		node_open_list.pop_back();
		
		binMap coverageArea;
		omap.esz(node->getX(), node->getY(), coverageArea, dilationradius, actionradius);			// get its coverage area
		binMap* filter = new binMap(coverageArea);								// copy it to the filter	
		filter->sub(processed);											// remove processed parts of the coverage area
		filter->removeUnconnected(node->getX(), node->getY());							// remove unconnected components
		filter->set(node->getX(), node->getY(), true);
		node->setArea(*filter);											// what remains its the area associated to the node
		seekChildNodes(omap, *node, coverageArea, *filter, min_cluster_size);					// seek for gateways in that area
		for(int i = 0; i< node->getNChildren(); i++){
			node_open_list.push_back(&node->getChildren(i));						// add nodes to the open list with the items found
			count++;		
		}
		processed.add(*filter);											// we add the new area to the processed zones 
	};

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
				for( k = 0, it = seq.begin(); it != seq.end() && k <= seq.size()/2; it++ , k++);		
				it--; k--;
				p.x = it->x; p.y = it->y;

				treeNode* newNode = new treeNode(nodetype); 						// new node
				newNode->setCell(p);									// set the position to the middle cell
				float d = EPS+sqrt(pow(((float)parent.getX())-p.x,2)+pow(((float)parent.getY())-p.y,2));// calculate the cost as the euclidean distance
				newNode->setLocalCost(d); 								// set local cost
				newNode->setGlobalCost(parent.getGlobalCost() + d); 					// set global cost
				parent.addChildren(*newNode);								// add the node to the tree

				count++;										
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
