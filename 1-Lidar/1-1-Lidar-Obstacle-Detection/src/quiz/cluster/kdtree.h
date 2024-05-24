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
	 * @brief Inserts a node into the tree using this recursive function.
	 * @param node   Current node in the K-D Tree to examine.
	 * @param depth	 Counter used to determine which axis to branch on.
	 * @param point  Value to assign the new `Node` to insert.
	 * @param id	 Counter indicating the sequential position of the new node,
	*/
	void insert(
		Node *&node,
		int depth,
		std::vector<float> point, 
		int id
	) {
		/** E1.3.3: Inserting a new `Node` into the tree. **/
		/* Traversing the tree until an "empty" node is found */
		// Determining which of the two coordinate axes to "split" on
		// i.e., we consider either the $x$- or $y$-axis value at this iteration
		uint axis = depth % 2;
		// Using the point value to determine where the node should be inserted
		if (node == NULL) {
			// CASE 1: Found an "empty" node,
			// Insert new `Node` into this location.~
			node = new Node(
				point,
				id
			);
		}
		// CASE 2: Determine which child node to branch to
		// Depending on the point value given, choose left or right
		else if (node->point[axis] > point[axis]) {
			// Value of point to insert is less than current node,
			// i.e., branch to left child node.
			insert(
				node->left,
				depth + 1, 
				point, 
				id
			);
		}
		else if (node->point[axis] < point[axis]) {
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
			// CANDO: Handle error with case(s).
		}
	}
	/** Inserts a new `Node` instance with given `value` into the K-D Tree.
	 * 
	 * This function relies on a call to the recursive `insert` function, which
	 * is overloaded with two additional parameters; the current `node` to
	 * examine, and a `depth` in the tree at which we are exploring. The `depth`
	 * is used to determine which of the two axes, either $x$- or $y$-axis, will
	 * be "looked at" to consider branching to the left- or to the right child
	 * node in the K-D Tree.
	 * 
	 * @brief Inserts the given `point` into the tree at the correct position.
	 * @param point  2D coordinate pair to insert into the K-D Tree.
	 * @param id	 Counter indicating the sequential order of the point to insert.
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
	/** Searches the K-D Tree and returns neighbouring points to `target`.
	 * 
	 * A nearest neighbour search is performed over the K-D Tree, which acts
	 * to partition the possible search space into smaller, more probable
	 * regions using the provided distance threshold (`distanceTol`).
	 * 
	 * @brief Searches the K-D Tree for neighbouring points to `target`.
	 * @param target	  2D point vector of node to search for in tree.
	 * @param distanceTol Distance tolerance (in metres) used to bisect search space.
	 * @returns Vector of node `id` values that are in proximity to `target`.
	 * 
	*/
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(
		std::vector<float> target, 
		float distanceTol
	) {
		std::vector<int> ids;
		return ids;
	}
};




