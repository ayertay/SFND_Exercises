/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct MyKdTree
{
	Node* root;

	MyKdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		insertHelper(&root, 0, point, id);
		// the function should create a new node and place correctly with in the root 

	}

	void insertHelper(Node **node, uint depth, std::vector<float> point, int id)
   {
      if(*node == NULL)
      {
        *node = new Node(point, id);
      }
      else
      {
        uint cd = depth % 3;
      	
      	if (point[cd] < ((*node)->point[cd]))
      	{
      		insertHelper(&((*node)->left), ++depth, point, id);
      	}
      	else
      	{
      		insertHelper(&((*node)->right), ++depth, point, id);
      	}
      }
   }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int i)
	{

		std::vector<int> ids;
		searchHelper(root, cloud, 0, distanceTol, ids, i);
		return ids;
	}

	void searchHelper(Node *node, typename pcl::PointCloud<PointT>::Ptr cloud, uint depth, float distanceTol, std::vector<int>& ids, int i)
	{
		std::vector<float> target;
		target.push_back(cloud->points[i].x);
        target.push_back(cloud->points[i].y);
        target.push_back(cloud->points[i].z);
		if (node != NULL)
		{
			//Box region
			if (((target[0] + distanceTol) >= (node->point[0])) && ((target[0] - distanceTol) <= (node->point[0])) && ((target[1] + distanceTol) >= (node->point[1])) && ((target[1] - distanceTol) <= (node->point[1])) && ((target[2] + distanceTol) >= (node->point[2])) && ((target[2] - distanceTol) <= (node->point[2])))
			{
				float distance = sqrt(pow((target[0] - (node->point[0])), 2) + pow((target[1] - (node->point[1])), 2) + pow((target[2] - (node->point[2])), 2));
				if (distance <= distanceTol)
					{
						ids.push_back((node->id));
					}
			}
			uint cd = depth % 3;
			if ((target[cd] - distanceTol) < (node->point[cd]))
			{
				searchHelper((node->left), cloud, ++depth, distanceTol, ids, i);
			}
			if ((target[cd] + distanceTol) > (node->point[cd]))
			{
				searchHelper((node->right), cloud, ++depth, distanceTol, ids, i);
			}

		}
		

	}
	

};




