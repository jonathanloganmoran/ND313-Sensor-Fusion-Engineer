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
	/** Recursive insert function for the K-D Tree.
	 * 
	 * Recursively performs a traversal of the K-D Tree structure;
	 * `insert` searches for an "empty" node in which the provided node
	 * data can be inserted into. This function assumes a 2D input value
	 * is given for the `point` argument. At each "depth" the coordinate
	 * axis to "split" alternates; either the $x$-axis or the $y$-axis
	 * will be considered for determining the splitting criteria at each
	 * iteration. In other words, the tree will be traversed by examining
	 * either the left- or the right sub-branches depending on the current
	 * "depth" and the value of the respective axis being considered.   
	 * 
	 * ~~TODO: How is recursion handled in the `insert` function?~~
	 * UPDATE: Recursion is handled by creating an overloaded `insert` function.
	 * TODO: How is the `getNewNode` translatable to the `KdTree` struct we have here?
	 * 
	 * @brief Inserts a node into the tree using this recursive function.
	 * @param node   Current node in the K-D Tree to examine.
	 * @param depth	 Counter used to determine which axis to branch on.
	 * @param point  Value to assign the new `Node` to insert.
	 * @param id	 Counter indicating the sequential position of the new node,
	 * 				 ~~TODO: is `id` the depth of the tree? Used to determine the
	 * 				 axis to split at the current iteration?~~
	 * 				UPDATE: `id` is a sequential counter for nodes in tree.
	 * 				UPDATE: new variable `depth` determines axis to split on.
	*/
	void insert(
		Node *&node,
		int depth,
		std::vector<float> point, 
		int id
	) {
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		/** E1.3.3: Inserting a new `Node` into the tree. **/
		/* Traversing the tree until an "empty" node is found */
		// Determining which of the two coordinate axes to "split" on
		// i.e., we consider either the $x$- or $y$-axis value at this iteration
		uint axis = depth % 2;
		// Using the point value to determine where the node should be inserted
		// ~~TODO: Switch the usage of `node` to be synonymous with "current node"~~
		// UPDATE: Using recursive function call to handle current node iteration
		if (node == NULL) {
			// CASE 1: Found an "empty" node,
			// ~~TODO: insert new `Node` into this location.~~
			node = new Node(
				point,
				id
			);
		}
		// CASE 2: Determine which child node to branch to
		// Depending on the point value given, choose left or right
		else if (node->point[axis] > point) {
			// Value of point to insert is less than current node,
			// i.e., branch to left child node.
			insert(
				node->left,
				depth + 1, 
				point, 
				id
			);
		}
		else if (node->point[axis] < point) {
			// Value of point to insert is greater than current node,
			// i.e., branch to right child node.
			insert(
				node->right,
				depth + 1,
				point, 
				id
			);
		}
		else {
			// ERROR; should not occur.
			// TODO: Handle error with case(s).
		}
	}
	/** Inserts a new `Node` instance with given `value` into the K-D Tree.
	 * 
	 * ~~TODO: How is recursion handled in this `insert` function?~~
	 * UPDATE: Recursion is handled by creating an overloaded `insert` function
	 * 		   which is called recursively to traverse the K-D Tree.
	 * ~~TODO: How is the `getNewNode` translatable to the `KdTree` struct we have here?~~
	 * UPDATE: We use an overloaded `insert` function which takes in a current `node`
	 * reference pointer and "updates" the node with our desired value by creating
	 * a new `Node` instance and updating the memory address of the de-referenced
	 * pointer to the new node (as performed in the recursive `insert` function).
	 * 
	 * @brief Inserts the given `point` into the tree at the correct position.
	 * @param point  2D coordinate pair to insert into the K-D Tree.
	 * @param id	 Counter indicating the sequential order of the point to insert,
	 * 				 ~~TODO: is `id` the depth of the tree? Used to determine the
	 * 				 axis to split at the current iteration?~~
	 * 	  			 UPDATE: `id` is a sequential counter for nodes in tree.
	 * 				 UPDATE: new variable `depth` determines axis to split on.
	*/
	void insert(
		std::vector<float> point,
		int id
	) {
		// Begin tree traversal at the initial depth
		int depth = 0;
		// Recursive function call to traverse the tree and insert a new node
		insert(
			this->root,
			depth,
			point,
			id
		);

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




