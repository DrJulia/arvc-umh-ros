#pragma once

#include <ros/ros.h>
#include <tf/tf.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <toro/treeoptimizer2.hh>

class graph_node; // forward declaration of this class, required pointers to it before declaration

/// *************************    EDGE    *********************************************
class graph_edge{
public:
	graph_edge(tf::Transform&, cv::Mat&, graph_node*);
	virtual ~graph_edge(){};

	tf::Transform constraint;
	cv::Mat constraint_covariance;
	graph_node* to_node;
};

/// *************************    NODE    *********************************************
class graph_node{
public:
	graph_node();
	virtual ~graph_node(){};

	void set_gridmap(const cv::Mat&);
	void set_disc_map(const cv::Mat&);
	void set_dist_map(const cv::Mat&);
	void set_features(const std::vector<cv::KeyPoint>& );
	void set_descriptors(const cv::Mat& );
	void set_initial_origin(const tf::Transform&);
	void set_optimized_origin(const tf::Transform&);
	void set_optimized_covariance(const cv::Mat&);
	void set_node_stamp(const ros::Time&);

	cv::Mat& 			get_gridmap();
	cv::Mat& 			get_disc_map();
	cv::Mat& 			get_dist_map();
	std::vector<cv::KeyPoint>& 	get_features();
	cv::Mat& 			get_descriptors();
	tf::Transform& 			get_initial_origin();
	tf::Transform& 			get_optimized_origin();
	cv::Mat& 			get_optimized_covariance();
	ros::Time&			get_node_stamp();
	int				get_id();
	void				set_id(int id);
 	void add_edge(const graph_edge&);
	std::list<graph_edge>& getEdges();
private:
	ros::Time map_stamp;	
	int node_id;
	cv::Mat gridmap;
	cv::Mat disc_map;
	cv::Mat dist_map;
	std::vector<cv::KeyPoint> features;
	cv::Mat descriptors;
	tf::Transform initial_origin;

	tf::Transform optimized_origin;
	cv::Mat optimized_covariance;

	std::list<graph_edge> edges;
};

inline void graph_node::set_gridmap		(const cv::Mat& m)			{gridmap = m;};
inline void graph_node::set_disc_map		(const cv::Mat& m)			{disc_map = m;};
inline void graph_node::set_dist_map		(const cv::Mat& m)			{dist_map = m;};
inline void graph_node::set_features		(const std::vector<cv::KeyPoint>& f)	{features = f;};
inline void graph_node::set_descriptors		(const cv::Mat& d)			{descriptors = d;};
inline void graph_node::set_initial_origin	(const tf::Transform& t)		{initial_origin = t;
											 optimized_origin = t;};
inline void graph_node::set_optimized_origin	(const tf::Transform& t)		{optimized_origin = t;};
inline void graph_node::set_optimized_covariance(const cv::Mat& c)			{optimized_covariance = c;};
inline void graph_node::set_node_stamp		(const ros::Time& t)			{map_stamp = t;};
inline void graph_node::set_id			(int id)				{node_id=id;};

inline cv::Mat& 			graph_node::get_gridmap()		{return gridmap;};
inline cv::Mat& 			graph_node::get_disc_map()		{return disc_map;};
inline cv::Mat& 			graph_node::get_dist_map()		{return dist_map;};
inline std::vector<cv::KeyPoint>& 	graph_node::get_features()		{return features;};
inline cv::Mat& 			graph_node::get_descriptors()		{return descriptors;};
inline tf::Transform& 			graph_node::get_initial_origin()	{return initial_origin;};
inline tf::Transform& 			graph_node::get_optimized_origin()	{return optimized_origin;};
inline cv::Mat& 			graph_node::get_optimized_covariance()	{return optimized_covariance;};
inline ros::Time& 			graph_node::get_node_stamp()		{return map_stamp;};
inline int				graph_node::get_id()			{return node_id;};

inline void graph_node::add_edge(const graph_edge& e)			{edges.push_back(e);};
inline std::list<graph_edge>& graph_node::getEdges()  			{return edges;};

/// *************************    GRAPH    *********************************************
class global_graph{
public:
	// constructor
	global_graph();
	virtual ~global_graph(){};

	void optimize();

	void add_node(graph_node*); 
	graph_node* get_last_node();
	std::vector<graph_node*>& getNodes();

	void rebuild();

private:
	bool restart_map_;
	bool initialized_;
	std::vector<graph_node*> nodes;
	AISNavigation::TreeOptimizer2 pg_;

};


inline graph_node* global_graph::get_last_node()		{return nodes.back();};
inline std::vector<graph_node*>& global_graph::getNodes()  	{return nodes;};



