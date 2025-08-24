#include "algorithm.h"
#include <vector>
#include <unordered_set>

using namespace std;

///////////////////////////////////////////////////////////////////////////////////////////
//
// Search Algorithm:  UC with Strict Expanded List
//
// Move Generator:  
//
////////////////////////////////////////////////////////////////////////////////////////////
string uc_explist(string const initialState, string const goalState, int& pathLength, int &numOfStateExpansions, int& maxQLength,
                               float &actualRunningTime, int &numOfDeletionsFromMiddleOfHeap, int &numOfLocalLoopsAvoided, int &numOfAttemptedNodeReExpansions){
                                 
   string path;
   clock_t startTime;
   
   numOfDeletionsFromMiddleOfHeap=0;
   numOfLocalLoopsAvoided=0;
   numOfAttemptedNodeReExpansions=0;
   maxQLength=0;
   numOfStateExpansions =0;
   actualRunningTime=0.0;  
   startTime = clock();
   
   struct UCComparator {
      bool operator()(Puzzle *p1, Puzzle *p2) {
         return p1->getPathLength() > p2->getPathLength();
      }
   };
   
   
   unordered_set<string> expandedList; // unordered set to use hashing
   vector<Puzzle*> puzzleQueue; //use pointers to save space
   
   Puzzle *startPuzzle = new Puzzle(initialState, goalState); //create start puzzle
   puzzleQueue.push_back(startPuzzle); //push it to vector (which will be used as min heap)
   
   while (!puzzleQueue.empty()) {

      // Track maximum queue length
      if ((int)puzzleQueue.size() > maxQLength) {
         maxQLength = (int)puzzleQueue.size();
      }
      
      // Get node with lowest path cost
      pop_heap(puzzleQueue.begin(), puzzleQueue.end(), UCComparator()); // moves front to back and keeps heap structure
      Puzzle* current = puzzleQueue.back();
      puzzleQueue.pop_back();
      
      string currentState = current->toString();
      
      // Check if we have reached the goal
      if (current->goalMatch()) {
         path = current->getPath();
         pathLength = current->getPathLength();
         
         // Clean memory
         delete current;
         for (Puzzle* p : puzzleQueue) {
            delete p;
         }
         
         actualRunningTime = ((float)(clock() - startTime)/CLOCKS_PER_SEC);
         return path;
      }
      
      // Check if already expanded
      if (expandedList.find(currentState) != expandedList.end()) { 
         numOfAttemptedNodeReExpansions++;
         delete current;
         continue;
      }
      
      // Add to expanded list and increment counter
      expandedList.insert(currentState);
      numOfStateExpansions++;
      
      // Generate successors in the order of up, right, down, left
      vector<Puzzle*> successors;
      
      if (current->canMoveUp()) {
         successors.push_back(current->moveUp());
      }
      if (current->canMoveRight()) {
         successors.push_back(current->moveRight());
      }
      if (current->canMoveDown()) {
         successors.push_back(current->moveDown());
      }
      if (current->canMoveLeft()) {
         successors.push_back(current->moveLeft());
      }
      
      // Add the valid successors to queue
      for (Puzzle* successor : successors) {
         string successorState = successor->toString();
   
         // Check if successor is already expanded - don't increment LOCAL_LOOPS_AVOIDED (expanded list prevents local loops entirely)
         if (expandedList.find(successorState) != expandedList.end()) {
            delete successor;
         } else {
            // Step 7: If descendant state already in Q, keep only shorter path to state in Q
            bool foundInQueue = false;
            for (auto it = puzzleQueue.begin(); it != puzzleQueue.end(); ++it) {
               if ((*it)->toString() == successorState) {
                  foundInQueue = true;
                  if (successor->getPathLength() < (*it)->getPathLength()) {
                     // New path is better, remove old one from middle of heap
                     delete *it;
                     puzzleQueue.erase(it);
                     numOfDeletionsFromMiddleOfHeap++;
                     make_heap(puzzleQueue.begin(), puzzleQueue.end(), UCComparator());
                     // Add the better successor
                     puzzleQueue.push_back(successor);
                     push_heap(puzzleQueue.begin(), puzzleQueue.end(), UCComparator());
                  } else {
                     // Old path is better or equal, discard new successor
                     delete successor;
                  }
                  break;
               }
            }
            if (!foundInQueue) {
               // State not in queue, add it
               puzzleQueue.push_back(successor);
               push_heap(puzzleQueue.begin(), puzzleQueue.end(), UCComparator());
            }
         }
      }
      
      delete current;
   }
   
   // clean up memory
   actualRunningTime = ((float)(clock() - startTime)/CLOCKS_PER_SEC);
   pathLength = 0;
   return ""; // we return empty string if no solution
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Search Algorithm:  A* with the Strict Expanded List
//
// Move Generator:  
//
////////////////////////////////////////////////////////////////////////////////////////////
string aStar_ExpandedList(string const initialState, string const goalState, int& pathLength, int &numOfStateExpansions, int& maxQLength,
                               float &actualRunningTime, int &numOfDeletionsFromMiddleOfHeap, int &numOfLocalLoopsAvoided, int &numOfAttemptedNodeReExpansions, heuristicFunction heuristic){
                                 
   string path;
   clock_t startTime;
   
   numOfDeletionsFromMiddleOfHeap=0;
   numOfLocalLoopsAvoided=0;
   numOfAttemptedNodeReExpansions=0;
   maxQLength=0;
   numOfStateExpansions =0;
   actualRunningTime=0.0;  
   startTime = clock();
   
   // comparator for A* uses lower f-cost with tie breaking criteria that priositises larger g-cost
   struct AStarComparator {
      bool operator()(Puzzle *p1, Puzzle *p2) {
         // Get f-costs
         int f1 = p1->getFCost();
         int f2 = p2->getFCost();
         
         if (f1 != f2) {
            return f1 > f2;
         }
         int g1 = p1->getGCost();
         int g2 = p2->getGCost();
         return g1 < g2;
      }
   };
   
   // Initialize data structures
   unordered_set<string> expandedList;
   vector<Puzzle*> puzzleQueue;
   
   Puzzle *startPuzzle = new Puzzle(initialState, goalState);
   startPuzzle->updateHCost(heuristic);
   startPuzzle->updateFCost();
   puzzleQueue.push_back(startPuzzle);
   
   while (!puzzleQueue.empty()) {
      // Track maximum queue length
      if ((int)puzzleQueue.size() > maxQLength) {
         maxQLength = (int)puzzleQueue.size();
      }
      
      // Get node with lowest f-cost
      pop_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
      Puzzle* current = puzzleQueue.back();
      puzzleQueue.pop_back();
      
      string currentState = current->toString();
      
      // Strict expanded list check
      if (expandedList.find(currentState) != expandedList.end()) {
         numOfAttemptedNodeReExpansions++;
         delete current;
         continue;
      }
      
      // Goal test
      if (current->goalMatch()) {
         path = current->getPath();
         pathLength = current->getPathLength();
         
         // Clean up
         delete current;
         for (Puzzle* p : puzzleQueue) {
            delete p;
         }
         
         actualRunningTime = ((float)(clock() - startTime)/CLOCKS_PER_SEC);
         return path;
      }
      
      // Add to expanded list
      expandedList.insert(currentState);
      numOfStateExpansions++;
      
      // Generate successors: Up, Right, Down, Left
      if (current->canMoveUp()) {
         Puzzle* successor = current->moveUp();
         string successorState = successor->toString();
         
         // Check if successor is already expanded - don't increment LOCAL_LOOPS_AVOIDED (expanded list prevents local loops entirely)
         if (expandedList.find(successorState) != expandedList.end()) {
            delete successor;
         } else {
            successor->updateHCost(heuristic);
            successor->updateFCost();
            
            // Step 7: If descendant state already in Q, keep only shorter path (compare f-cost, then g-cost)
            bool foundInQueue = false;
            for (auto it = puzzleQueue.begin(); it != puzzleQueue.end(); ++it) {
               if ((*it)->toString() == successorState) {
                  foundInQueue = true;
                  bool newIsBetter = false;
                  if (successor->getFCost() < (*it)->getFCost()) {
                     newIsBetter = true;
                  } else if (successor->getFCost() == (*it)->getFCost() && successor->getGCost() > (*it)->getGCost()) {
                     newIsBetter = true;
                  }
                  
                  if (newIsBetter) {
                     // New path is better, remove old one from middle of heap
                     delete *it;
                     puzzleQueue.erase(it);
                     numOfDeletionsFromMiddleOfHeap++;
                     make_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
                     puzzleQueue.push_back(successor);
                     push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
                  } else {
                     // Old path is better or equal, discard new successor
                     delete successor;
                  }
                  break;
               }
            }
            if (!foundInQueue) {
               // State not in queue, add it
               puzzleQueue.push_back(successor);
               push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
            }
         }
      }
      
      if (current->canMoveRight()) {
         Puzzle* successor = current->moveRight();
         string successorState = successor->toString();
         
         if (expandedList.find(successorState) != expandedList.end()) {
            delete successor;
         } else {
            successor->updateHCost(heuristic);
            successor->updateFCost();
            
            bool foundInQueue = false;
            for (auto it = puzzleQueue.begin(); it != puzzleQueue.end(); ++it) {
               if ((*it)->toString() == successorState) {
                  foundInQueue = true;
                  bool newIsBetter = false;
                  if (successor->getFCost() < (*it)->getFCost()) {
                     newIsBetter = true;
                  } else if (successor->getFCost() == (*it)->getFCost() && successor->getGCost() > (*it)->getGCost()) {
                     newIsBetter = true;
                  }
                  
                  if (newIsBetter) {
                     delete *it;
                     puzzleQueue.erase(it);
                     numOfDeletionsFromMiddleOfHeap++;
                     make_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
                     puzzleQueue.push_back(successor);
                     push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
                  } else {
                     delete successor;
                  }
                  break;
               }
            }
            if (!foundInQueue) {
               puzzleQueue.push_back(successor);
               push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
            }
         }
      }
      
      if (current->canMoveDown()) {
         Puzzle* successor = current->moveDown();
         string successorState = successor->toString();
         
         if (expandedList.find(successorState) != expandedList.end()) {
            delete successor;
         } else {
            successor->updateHCost(heuristic);
            successor->updateFCost();
            
            bool foundInQueue = false;
            for (auto it = puzzleQueue.begin(); it != puzzleQueue.end(); ++it) {
               if ((*it)->toString() == successorState) {
                  foundInQueue = true;
                  bool newIsBetter = false;
                  if (successor->getFCost() < (*it)->getFCost()) {
                     newIsBetter = true;
                  } else if (successor->getFCost() == (*it)->getFCost() && successor->getGCost() > (*it)->getGCost()) {
                     newIsBetter = true;
                  }
                  
                  if (newIsBetter) {
                     delete *it;
                     puzzleQueue.erase(it);
                     numOfDeletionsFromMiddleOfHeap++;
                     make_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
                     puzzleQueue.push_back(successor);
                     push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
                  } else {
                     delete successor;
                  }
                  break;
               }
            }
            if (!foundInQueue) {
               puzzleQueue.push_back(successor);
               push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
            }
         }
      }
      
      if (current->canMoveLeft()) {
         Puzzle* successor = current->moveLeft();
         string successorState = successor->toString();
         
         if (expandedList.find(successorState) != expandedList.end()) {
            delete successor;
         } else {
            successor->updateHCost(heuristic);
            successor->updateFCost();
            
            bool foundInQueue = false;
            for (auto it = puzzleQueue.begin(); it != puzzleQueue.end(); ++it) {
               if ((*it)->toString() == successorState) {
                  foundInQueue = true;
                  bool newIsBetter = false;
                  if (successor->getFCost() < (*it)->getFCost()) {
                     newIsBetter = true;
                  } else if (successor->getFCost() == (*it)->getFCost() && successor->getGCost() > (*it)->getGCost()) {
                     newIsBetter = true;
                  }
                  
                  if (newIsBetter) {
                     delete *it;
                     puzzleQueue.erase(it);
                     numOfDeletionsFromMiddleOfHeap++;
                     make_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
                     puzzleQueue.push_back(successor);
                     push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
                  } else {
                     delete successor;
                  }
                  break;
               }
            }
            if (!foundInQueue) {
               puzzleQueue.push_back(successor);
               push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
            }
         }
      }
      
      delete current;
   }
   
   // if we dont find a solution we retuurn emppty string
   actualRunningTime = ((float)(clock() - startTime)/CLOCKS_PER_SEC);
   pathLength = 0;
   return "";
}