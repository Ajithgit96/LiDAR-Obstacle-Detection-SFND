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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	//traversing through out the tree for every incoming node, until we reach a null node
	//and we insert new node in that null node.
	//this insethelper function will terminate when it reaches a null node.
	void inserthelper(Node** node, uint depth, std::vector<float> point, int id){
		if(*node == NULL){
			*node = new Node(point, id);
		}
		else{
/*-----for 2D KD Tree
			//calulcating depth
			uint cd = depth%2;
			if(point[cd] < ((*node)->point[cd])){
				inserthelper(&((*node)->left), depth+1, point, id);
			}
			else{
				inserthelper(&((*node)->right), depth+1, point, id);
			}*/

			//----for 3D KD Tree
			//calulcating depth
			uint cd = depth%3;
			if(point[cd] < ((*node)->point[cd])){
				inserthelper(&((*node)->left), depth+1, point, id);
			}
			else{
				inserthelper(&((*node)->right), depth+1, point, id);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		inserthelper(&root, 0, point,id);
		// the function should create a new node and place correctly with in the root 

	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids){

/*------2D KD Tree search
		if(node != NULL){
			if((node->point[0]>=(target[0]-distanceTol)&&node->point[0]<=(target[0]+distanceTol))&&(node->point[1]>=(target[1]-distanceTol)&&node->point[1]<=(target[1]+distanceTol))){
				float distance = sqrt(((node->point[0]-target[0])*(node->point[0]-target[0]))+((node->point[1]-target[1])*(node->point[1]-target[1])));
				if(distance <= distanceTol){
					ids.push_back(node->id);
				}
			}

			//uint cd = depth%2;
			if(node->point[depth%2]>target[depth%2]-distanceTol){
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
			if(node->point[depth%2]<target[depth%2]+distanceTol){
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}*/

		//3D KD Tree search
		if(node != NULL){
			if((node->point[0]>=(target[0]-distanceTol)&&node->point[0]<=(target[0]+distanceTol))&&(node->point[1]>=(target[1]-distanceTol)&&node->point[1]<=(target[1]+distanceTol))&&(node->point[2]>=(target[2]-distanceTol)&&node->point[2]<=(target[2]+distanceTol))){
				float distance = sqrt(((node->point[0]-target[0])*(node->point[0]-target[0]))+((node->point[1]-target[1])*(node->point[1]-target[1]))+((node->point[2]-target[2])*(node->point[2]-target[2])));
				if(distance <= distanceTol){
					ids.push_back(node->id);
				}
			}

			if(node->point[depth%3]>target[depth%3]-distanceTol){
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
			if(node->point[depth%3]<target[depth%3]+distanceTol){
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0 , distanceTol, ids);
		return ids;
	}
	

};




