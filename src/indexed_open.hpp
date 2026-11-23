#pragma once
#include "planner.hpp"

// The label owns its slot. unordered_map keeps label addresses stable on rehash.
// A node can change key in either direction or leave OPEN before being popped.
/// Tracks a search label's current heap index or the absent sentinel when it is outside the queue.
struct HeapSlot {
    static constexpr size_t absent = std::numeric_limits<size_t>::max();
    size_t index = absent;
};

/// Maintains the OPEN set as a min-heap with direct access to each queued node through its slot.
class IndexedOpen {
    /// Pairs a node and its priority key with the label-owned slot updated whenever the entry moves.
    struct Entry {
        Key key;
        Node node;
        HeapSlot *slot;
        /// Order entries by priority key, breaking ties with the node ID of this entry and other.
        bool operator<(const Entry &other) const {
            return std::tie(key, node) < std::tie(other.key, other.node);
        }
    };
    std::vector<Entry> heap;

    /// Swap the entries at heap indices a and b and update their external slots.
    void exchange(size_t a, size_t b) {
        std::swap(heap[a], heap[b]);
        heap[a].slot->index = a;
        heap[b].slot->index = b;
    }
    /// Move the entry at heap index i toward the root until its parent has equal or higher priority.
    void up(size_t i) {
        while (i && heap[i] < heap[(i - 1) / 2]) {
            size_t parent = (i - 1) / 2;
            exchange(i, parent);
            i = parent;
        }
    }
    /// Move the entry at heap index i toward the leaves until neither child has higher priority.
    void down(size_t i) {
        while (2*i + 1 < heap.size()) {
            size_t child = 2*i + 1;
            if (child + 1 < heap.size() && heap[child + 1] < heap[child]) ++child;
            if (!(heap[child] < heap[i])) break;
            exchange(i, child);
            i = child;
        }
    }
    /// Remove the entry at heap index i, mark its slot absent and restore heap order.
    void remove_at(size_t i) {
        exchange(i, heap.size()-1);
        heap.back().slot->index = HeapSlot::absent;
        heap.pop_back();
        if (i == heap.size()) return;
        if (i && heap[i] < heap[(i-1)/2]) up(i);
        else down(i);
    }

public:
    /// Return true when OPEN contains no entries.
    bool empty() const { return heap.empty(); }
    /// Return the lowest priority key or an infinite key for an empty heap. Record the peek in ops.
    Key minimum(QueueOps &ops) const {
        ++ops.peek;
        return heap.empty() ? Key{Cost::inf(), Cost::inf()} : heap.front().key;
    }
    /// Give node n the priority key. Its label owns slot, which must remain valid while queued.
    /// Count insertions and priority changes in ops.
    void set(Node n, Key key, HeapSlot &slot, QueueOps &ops) {
        size_t i = slot.index;
        if (i == HeapSlot::absent) {
            ++ops.insert;
            slot.index = heap.size();
            heap.push_back({key,n,&slot});
            up(slot.index);
            return;
        }
        Key old = heap[i].key;
        if (old == key) return;
        ++ops.change;
        heap[i].key = key;
        if (key < old) up(i);
        else down(i);
    }
    /// Remove the entry tracked by slot if present and count the removal in ops.
    void erase(HeapSlot &slot, QueueOps &ops) {
        if (slot.index != HeapSlot::absent) {
            ++ops.erase;
            remove_at(slot.index);
        }
    }
    /// Remove and return the minimum node and its key from a nonempty heap. Record the extraction in ops.
    std::pair<Node,Key> pop(QueueOps &ops) {
        ++ops.pop;
        auto first = std::make_pair(heap.front().node, heap.front().key);
        remove_at(0);
        return first;
    }
};
