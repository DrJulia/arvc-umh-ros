
#ifndef ___TREE_NODE__
#define ___TREE_NODE__

#include <vector>
#include <binMap.h>
#include <geometry_msgs/Point.h>
#include <gridMapInterface.h>

namespace treeNodeTypes{
	static const int GATEWAY_NODE = 0;
	static const int LEAF_NODE = 1;
	static const int LEAF_TYPE_FRONTIER =11;
	static const int LEAF_TYPE_PRECISE_POSE = 12;
	static const int LEAF_TYPE_IMPRECISE_POSE = 13;
}

class treeNode{

private:
	treeNode* parent_;
	std::vector<treeNode*> children_;
	
	int node_type_;
	int size_;

	int x_;
	int y_;
	binMap* area_;

	double localCost_;
	double globalCost_;

	double areaValue_;
	double cumulatedAreaValue_;

	double profit_;


public:
	treeNode(int type);
	virtual ~treeNode();

	void setParent(treeNode& parent);
	int getNChildren() const;
	void addChildren(treeNode& child);
	treeNode& getChildren(int number) const;
	treeNode& getParent() const;
	void eraseChild(treeNode*);

	void setCell(geometry_msgs::Point rcell);
	geometry_msgs::Point getCell() const;
	int getX() const;
	int getY() const;

	void setArea(binMap& area);
	binMap& getArea() const;

	void setAreaValue(double val);
	double getAreaValue() const;
	void setCumulatedAreaValue(double val);
	double getCumulatedAreaValue() const;

	void setSize(int size);
	int getSize() const;

	void setGlobalCost(double cost);
	double getGlobalCost() const;
	void setLocalCost(double cost);
	double getLocalCost() const;

	double getProfit() const;
	void setProfit(double profit);

	int getNodeType() const;
	bool isLeaf() const;
	bool isRoot() const;

	/// find the node of maximum profit in the tree
	const treeNode* findMaxProfitNode(double& maxprofit);	

	static bool orderByValue(const treeNode* a, const treeNode* b);
	static bool orderByCost(const treeNode* a, const treeNode* b);
	static bool orderByProfit(const treeNode* a, const treeNode* b);

	/// creates a partition of the environment in a tree
	static treeNode* createTreePartitioning(const gridMapInterface& omap, const geometry_msgs::Point& rcell, int dilationradius, int actionradius, int sensorradius, int min_cluster_size);
	/// auxiliary function to find new nodes to add to the tree
	static void seekChildNodes(const gridMapInterface& omap, treeNode& current, binMap& area, binMap& filter, int min_cluster_size);
	/// clusters a binmap, used to gateway point finding 
	static int cluster(const binMap& map, treeNode& parent, int nodetype, int min_cluster_size);
	/// auxiliary recursive clustering function
	static bool clustering(const int &i, const int &j, binMap& aux, const binMap& map, std::list<point> &seq, bool backfront);

};

inline void treeNode::setParent(treeNode& p)				{parent_ = &p;}
inline int treeNode::getNChildren() const 				{return children_.size();}
inline void treeNode::setCell(geometry_msgs::Point rcell)		{x_ = rcell.x; y_ = rcell.y;}
inline geometry_msgs::Point treeNode::getCell()	const			{geometry_msgs::Point p; p.x=x_; p.y=y_; return p;}
inline int treeNode::getX() const 					{return x_;}
inline int treeNode::getY() const 					{return y_;}
inline void treeNode::setArea(binMap& area)				{area_ = &area;}
inline binMap& treeNode::getArea() const				{return *area_;}
inline void treeNode::setAreaValue(double val)				{areaValue_ = val;}
inline double treeNode::getAreaValue() const				{return areaValue_;}
inline void treeNode::setCumulatedAreaValue(double val)			{cumulatedAreaValue_ = val;}
inline double treeNode::getCumulatedAreaValue() const			{return cumulatedAreaValue_;}
inline void treeNode::setGlobalCost(double cost)			{globalCost_ = cost;}
inline double treeNode::getGlobalCost() const 				{return globalCost_;}
inline void treeNode::setLocalCost(double cost)				{localCost_ = cost;}
inline double treeNode::getLocalCost() const 				{return localCost_;}
inline void treeNode::setProfit(double prof)				{profit_ = prof;}
inline double treeNode::getProfit() const 				{return profit_;}
inline int treeNode::getNodeType() const				{return node_type_;}
inline bool treeNode::isLeaf() const 					{return (node_type_ != treeNodeTypes::GATEWAY_NODE);}
inline bool treeNode::isRoot() const					{return(parent_==0);}
inline void treeNode::setSize(int s) 					{size_ =s;}
inline int treeNode::getSize() const					{return size_;}

#endif

