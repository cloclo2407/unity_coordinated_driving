using System;
using System.Collections;
using System.Collections.Generic;

public class PriorityQueue : IEnumerable<StateNode>
{
    //Class implementing PriorityQueues as binary min heaps, for use with StateNode-objects
    private List<StateNode> bin_min_heap = new List<StateNode>(); //The binary heap as List, implementing the priority queue according
                                                  //to minimum value of T objects
    public int Count => bin_min_heap.Count;
    
    //Instead of defining an enumerator for PiorityQueue, I reuse the enumerator of List<StateNode>
    // With an enumerator for PriorityQueue, we can do fun things such as use PriorityQueues in foreach-statements
    // Implement IEnumerable<StateNode>
    public IEnumerator<StateNode> GetEnumerator()
    { return bin_min_heap.GetEnumerator(); }
    // Implement non-generic IEnumerable
    IEnumerator IEnumerable.GetEnumerator()
    { return GetEnumerator(); }
    // Both of these^ are necessary for the compiler to not get angry with us.
    
    public void Enqueue(StateNode node) //When we enqueue, we add new node to last element of bin_min_heap List and then bubble 
    {                           // it up to its correct position in the bin_min_heap tree.
        bin_min_heap.Add(node);
        BubbleUp(bin_min_heap.Count - 1);
    }

    public StateNode Dequeue() //When we dequeue, we pop the element at root of heap and then place the last element in
    {                  // bin_min_heap List at root. We then bubble it down to its correct position in the tree, filling '
                       // root with appropriate minmimum cost node in the process
        if (bin_min_heap.Count == 0)
            throw new InvalidOperationException("Priority queue is empty.");

        StateNode root_node = bin_min_heap[0];
        bin_min_heap[0] = bin_min_heap[bin_min_heap.Count - 1]; // Move last element to root
        bin_min_heap.RemoveAt(bin_min_heap.Count - 1);

        if (bin_min_heap.Count > 0)
            BubbleDown(0);

        return root_node;
    }

    /*
    public T Peek()
    {
        if (heap.Count == 0)
            throw new InvalidOperationException("The priority queue is empty.");
        return heap[0];
    }
    */

    public bool Contains(StateNode sn)
    {
        return bin_min_heap.Contains(sn);
    }

    private void BubbleUp(int index) //Move node at index up to parent position in case it has higher priority than 
    {                                // its parent. Keep going until bin_min_heap is sorted.
        while (index > 0)
        {
            int parentIndex = (index - 1) / 2;
            if (bin_min_heap[index].CompareTo(bin_min_heap[parentIndex]) >= 0) 
                break;

            Swap(index, parentIndex);
            index = parentIndex;
        }
    }

    private void BubbleDown(int index) //Move node at index down to a child position if it has lower priority
    {                                  // than that child. Keep going until bin_min_heap is sorted.
        int lastIndex = bin_min_heap.Count - 1;
        while (true)
        {
            int leftChild = 2 * index + 1;
            int rightChild = 2 * index + 2;
            int smallest = index;

            if (leftChild <= lastIndex && bin_min_heap[leftChild].CompareTo(bin_min_heap[smallest]) < 0)
                smallest = leftChild;

            if (rightChild <= lastIndex && bin_min_heap[rightChild].CompareTo(bin_min_heap[smallest]) < 0)
                smallest = rightChild;

            if (smallest == index)
                break;

            Swap(index, smallest);
            index = smallest;
        }
    }

    private void Swap(int i, int j)
    {
        var temp = bin_min_heap[i];
        bin_min_heap[i] = bin_min_heap[j];
        bin_min_heap[j] = temp;
    }
}