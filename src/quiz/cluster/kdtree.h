/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(std::vector<float> point, int id)
	{
		// insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}

	void insertHelper(Node **node, int depth, std::vector<float> point, int id){
		// Use point size considering data consistency over its dimension
		int coord_id = depth % point.size();

		if(*node == NULL){
			*node = new Node(point, id);
		}
		else{
          if (point[coord_id] > (*node)->point[coord_id]){
              insertHelper(&((*node)->right), depth + 1, point, id);
          }
          else{
              insertHelper(&((*node)->left), depth + 1, point, id);
          }
        }
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, distanceTol, 0, ids);
		return ids;
	}

	void searchHelper(std::vector<float> target, Node* node, float distanceTol, int depth, std::vector<int> &ids)
	{
		int coord_id = depth % root->point.size();
		if (node != NULL){
			
			bool check = true; 
			for (size_t i = 0; i < target.size(); i++) { 
				check &= std::abs(node->point[i] - target[i]) <= distanceTol; 
			}
			if (check){
                float distance = 0.f;
                for (size_t i = 0; i < target.size(); i++) {
                    distance += (target[i] - node->point[i]) * (target[i] - node->point[i]);
                } 
                distance = sqrt(distance);
                if (distance <= distanceTol){
                    ids.push_back(node->id);
                }
            }
          	
			if ((target[coord_id] - distanceTol) < node->point[coord_id]){
				searchHelper(target, node->left, distanceTol, depth+1, ids);
			}
          	if ((target[coord_id] + distanceTol) > node->point[coord_id]){
				searchHelper(target, node->right, distanceTol, depth+1, ids);
			}
		}
	}
	

};