#include "algorithm.h"
#include <vector>
#include <unordered_set>
#include <omp.h>

using namespace std;

enum class CompareType {
    PATH_LENGTH,
    F_COST
};

struct DuplicateResult {
    bool found;
    int index;
    bool newIsBetter;
    
    DuplicateResult() : found(false), index(-1), newIsBetter(false) {}
};

DuplicateResult checkQueueForDuplicateParallel(vector<Puzzle*>& puzzleQueue, Puzzle*& newPuzzle, CompareType compareType) {
    DuplicateResult result;
    const string& newState = newPuzzle->toString();
    const int queueSize = static_cast<int>(puzzleQueue.size());
    
    int foundIndex = -1;
    bool newIsBetter = false;
    
    #pragma omp parallel shared(foundIndex, newIsBetter)
    {
        int localFoundIndex = -1;
        bool localNewIsBetter = false;
        
        #pragma omp for nowait
        for (int i = 0; i < queueSize; i++) {
            if (foundIndex != -1) continue;
            
            if (puzzleQueue[i]->toString() == newState) {
                localFoundIndex = i;
                
                if (compareType == CompareType::PATH_LENGTH) {
                    localNewIsBetter = (newPuzzle->getPathLength() < puzzleQueue[i]->getPathLength());
                } else {
                    localNewIsBetter = (newPuzzle->getFCost() < puzzleQueue[i]->getFCost());
                }
                
                #pragma omp critical
                {
                    if (foundIndex == -1) {
                        foundIndex = localFoundIndex;
                        newIsBetter = localNewIsBetter;
                    }
                }
            }
        }
    }
    
    result.found = (foundIndex != -1);
    result.index = foundIndex;
    result.newIsBetter = newIsBetter;
    return result;
}

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
   
         // Check if successor is already expanded 
         if (expandedList.find(successorState) != expandedList.end()) {
            numOfAttemptedNodeReExpansions++;
            delete successor;
         } else {
            // Step 7: If descendant state already in Q, keep only shorter path to state in Q
            DuplicateResult dupResult = checkQueueForDuplicateParallel(puzzleQueue, successor, CompareType::PATH_LENGTH);
            
            if (dupResult.found) {
               if (dupResult.newIsBetter) {
                  // New path is better, remove old one from middle of heap
                  delete puzzleQueue[dupResult.index];
                  puzzleQueue.erase(puzzleQueue.begin() + dupResult.index);
                  numOfDeletionsFromMiddleOfHeap++;
                  make_heap(puzzleQueue.begin(), puzzleQueue.end(), UCComparator());
                  // Add the better successor
                  puzzleQueue.push_back(successor);
                  push_heap(puzzleQueue.begin(), puzzleQueue.end(), UCComparator());
               } else {
                  // Old path is better or equal, discard new successor
                  delete successor;
               }
            } else {
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
         
         successor->updateHCost(heuristic);
         successor->updateFCost();
         
         // Step 7: Check queue first - If descendant state already in Q, keep only one with lower f-cost
         DuplicateResult dupResult = checkQueueForDuplicateParallel(puzzleQueue, successor, CompareType::F_COST);
         
         if (dupResult.found) {
            if (dupResult.newIsBetter) {
               // New path is better, remove old one from middle of heap
               delete puzzleQueue[dupResult.index];
               puzzleQueue.erase(puzzleQueue.begin() + dupResult.index);
               numOfDeletionsFromMiddleOfHeap++;
               make_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
               puzzleQueue.push_back(successor);
               push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
            } else {
               // Old path is better or equal, discard new successor
               delete successor;
            }
         } else {
            // Not in queue, check if already expanded
            if (expandedList.find(successorState) != expandedList.end()) {
               numOfAttemptedNodeReExpansions++;
               delete successor;
            } else {
               // State not in queue or expanded, add it
               puzzleQueue.push_back(successor);
               push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
            }
         }
      }
      
      if (current->canMoveRight()) {
         Puzzle* successor = current->moveRight();
         string successorState = successor->toString();
         
         successor->updateHCost(heuristic);
         successor->updateFCost();
         
         DuplicateResult dupResult = checkQueueForDuplicateParallel(puzzleQueue, successor, CompareType::F_COST);
         
         if (dupResult.found) {
            if (dupResult.newIsBetter) {
               delete puzzleQueue[dupResult.index];
               puzzleQueue.erase(puzzleQueue.begin() + dupResult.index);
               numOfDeletionsFromMiddleOfHeap++;
               make_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
               puzzleQueue.push_back(successor);
               push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
            } else {
               delete successor;
            }
         } else {
            if (expandedList.find(successorState) != expandedList.end()) {
               numOfAttemptedNodeReExpansions++;
               delete successor;
            } else {
               puzzleQueue.push_back(successor);
               push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
            }
         }
      }
      
      if (current->canMoveDown()) {
         Puzzle* successor = current->moveDown();
         string successorState = successor->toString();
         
         successor->updateHCost(heuristic);
         successor->updateFCost();
         
         DuplicateResult dupResult = checkQueueForDuplicateParallel(puzzleQueue, successor, CompareType::F_COST);
         
         if (dupResult.found) {
            if (dupResult.newIsBetter) {
               delete puzzleQueue[dupResult.index];
               puzzleQueue.erase(puzzleQueue.begin() + dupResult.index);
               numOfDeletionsFromMiddleOfHeap++;
               make_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
               puzzleQueue.push_back(successor);
               push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
            } else {
               delete successor;
            }
         } else {
            if (expandedList.find(successorState) != expandedList.end()) {
               numOfAttemptedNodeReExpansions++;
               delete successor;
            } else {
               puzzleQueue.push_back(successor);
               push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
            }
         }
      }
      
      if (current->canMoveLeft()) {
         Puzzle* successor = current->moveLeft();
         string successorState = successor->toString();
         
         successor->updateHCost(heuristic);
         successor->updateFCost();
         
         DuplicateResult dupResult = checkQueueForDuplicateParallel(puzzleQueue, successor, CompareType::F_COST);
         
         if (dupResult.found) {
            if (dupResult.newIsBetter) {
               delete puzzleQueue[dupResult.index];
               puzzleQueue.erase(puzzleQueue.begin() + dupResult.index);
               numOfDeletionsFromMiddleOfHeap++;
               make_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
               puzzleQueue.push_back(successor);
               push_heap(puzzleQueue.begin(), puzzleQueue.end(), AStarComparator());
            } else {
               delete successor;
            }
         } else {
            if (expandedList.find(successorState) != expandedList.end()) {
               numOfAttemptedNodeReExpansions++;
               delete successor;
            } else {
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