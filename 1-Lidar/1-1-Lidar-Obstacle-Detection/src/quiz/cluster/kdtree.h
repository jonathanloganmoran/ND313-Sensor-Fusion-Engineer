/* ----------------------------------------------------------------------------
  * Project "1.1: LiDAR Obstacle Detection"
  * Authors     : Aaron Brown et al.
  *
  * Modified by : Jonathan Logan Moran (jonathan.moran107@gmail.com).
  *
  * Purpose of this file: Implements quiz questions related to the K-D Tree.
  * 		This corresponds to Exercises 1.3.3-5 of Course 1: Lidar in the
  *         Sensor Fusion Nanodegree offered by Udacity.
  * ---------------------------------------------------------------------------
  */

/** TODO: 
 * -[ ] Use 2-char vertical whitespacing for all but trailing indentation;
 * -[ ] Add Doxygen-style function / struct definiton comments;
 * -[x] Follow Google C++ Style Guide for syntax / styling;
 * -[x] Add file header comment;
 * -[x] Use 4-char vertical whitespacing for trailing indentation;
 * 		e.g., constructor initialiser lists in `struct` definitions. 
*/
#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node {
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;
	Node(std::vector<float> arr, int setId)
	:	point(arr), 
		id(setId), 
		left(NULL), 
		right(NULL) {}
	~Node() {
		delete left;
		delete right;
	}
};


struct KdTree {
	Node* root;
	KdTree()
		: root(NULL) {}
	~KdTree() {
		delete root;
	}
	/** Inserts a new `Node` instance with given `value` into the K-D Tree.
	 * 
	 * @brief Inserts a new `Node` with given value (`point`) into the tree.
	 * @param point  Value to assign the new `Node` to insert.
	 * @param id	 Counter indicating the sequential position of the new node.
	*/
	void insert(
		std::vector<float> point, 
		int id
	) {
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(
		std::vector<float> target, 
		float distanceTol
	) {
		std::vector<int> ids;
		return ids;
	}
};




