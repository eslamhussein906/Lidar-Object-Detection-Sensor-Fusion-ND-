#ifndef KD_Tree
#define KD_Tree
#include "render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;
	// Node constructor 
	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template <typename PointT>
struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}
	void insert_helper(Node **nodePtr ,int depth , PointT point ,int id){
		if(*(nodePtr)== NULL){
			std::vector<float> PointVector (point.data , point.data +3);
			*(nodePtr) = new Node(PointVector,id);
		}
		else{
			unsigned int current_depth = depth % 3;
			// compare between conent of inserted point and content of current node 
			if(point.data[current_depth]< ((*nodePtr)->point[current_depth]))
				insert_helper(&((*nodePtr)->left) ,depth+1 ,point ,id);
			else
				insert_helper(&((*nodePtr)->right) ,depth+1 ,point ,id);
			
		}

	}
	void insertCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
	{		
		// the function should create a new node and place correctly with in the root 
		for(unsigned int ID =0 ; ID < cloud->points.size();ID++){
			insert_helper(&root,0,cloud->points[ID],ID);
		}
		

	}
	
	void search_helper(Node* node, PointT target ,int depth ,float distanceTol,std::vector<int>& ids){
		// check that the node point in within the target box bounded by 2*distanceTol
		if(node!=NULL){
            // check that the point is inside the target Cube of Not
			if((node->point[0] >= (target.data[0]-distanceTol)&&(node->point[0]) <= (target.data[0]+ distanceTol))\
            && (node->point[1] >= (target.data[1]-distanceTol)&&(node->point[1]) <= (target.data[1]+ distanceTol))\
            && (node->point[2] >= (target.data[2]-distanceTol)&&(node->point[2]) <= (target.data[2]+ distanceTol)))
			{
				float dist = sqrt((target.data[0]-node->point[0])*(target.data[0]-node->point[0])\
                                + (target.data[1]-node->point[1])*(target.data[1]-node->point[1])\
                                + (target.data[2]-node->point[2])*(target.data[2]-node->point[2]));
				if(dist <= distanceTol)
					ids.push_back(node->id);
			}
			// the box in the left region of this point, so we call the left node of this node 
			if((target.data[depth%3]-distanceTol) < node->point[depth%3])
				search_helper(node->left,target ,depth+1 ,distanceTol,ids);
			// the box in the right region of this point, so we call the right node of this node 
			if((target.data[depth%3]+distanceTol) >= node->point[depth%3])
				search_helper(node->right,target ,depth+1 ,distanceTol,ids);

		}

	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(root ,target,0,distanceTol,ids);
		return ids;
	}
	

};

#endif


