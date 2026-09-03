import cslam.lidar_pr.scancontext_utils as sc_utils
import numpy as np
from scipy import spatial

class ScanContextMatching(object):
    """Nearest Neighbor matching of description vectors
    """

    def __init__(self, shape=[20,60], num_candidates=10, threshold=0.15): 
        """ Initialization
            Default configs are the same as in the original paper 

        """
        self.shape = shape
        self.num_candidates = num_candidates
        self.threshold = threshold

        self.scancontexts = np.zeros((1000, self.shape[0], self.shape[1]))
        self.ringkeys = np.zeros((1000, self.shape[0]))
        self.items = dict()
        self.nb_items = 0

    def add_item(self, descriptor, item):
        """Add item to the matching list

        Args:
            descriptor (np.array): descriptor
            item: identification info (e.g., int)
        """
        sc = descriptor.reshape(self.shape)

        if self.nb_items >= len(self.ringkeys):
            self.scancontexts.resize((2 * len(self.scancontexts), self.shape[0], self.shape[1]),
                                 refcheck=False)
            self.ringkeys.resize((2 * len(self.ringkeys), self.shape[0]),
                                 refcheck=False)
                                 
        rk = sc_utils.sc2rk(sc)

        self.scancontexts[self.nb_items] = sc
        self.ringkeys[self.nb_items] = rk
        self.items[self.nb_items] = item

        self.nb_items = self.nb_items + 1

    def search(self, query, k):
        """Search for nearest neighbors

        Args:
            query (np.array): descriptor to match
            k (int): number of best matches to return

        Returns:
            list(int), list(float): best matches and their similarities,
                sorted by decreasing similarity, at most k of them
        """
        if self.nb_items < 1:
            return [None], [None]

        # step 1: ring-key KD-tree prefilter.
        # Ask for at most nb_items neighbours - scipy pads the result with the
        # out-of-range index nb_items (and infinite distance) when asked for
        # more than the tree holds, and self.scancontexts is preallocated, so
        # those pads would otherwise be scored as all-zero descriptors.
        nb_candidates = min(self.num_candidates, self.nb_items)
        ringkey_history = np.array(self.ringkeys[:self.nb_items])
        ringkey_tree = spatial.KDTree(ringkey_history)

        query_sc = query.reshape(self.shape)
        ringkey_query = sc_utils.sc2rk(query_sc)
        _, nncandidates_idx = ringkey_tree.query(ringkey_query, k=nb_candidates)

        # step 2: full scan-context distance on the prefiltered candidates.
        # All of them are scored and kept, not just the single best: callers
        # such as LoopClosureSparseMatching.match_local_loop_closures() walk
        # the list to skip matches that are too close in time to be useful as
        # loop closures, which needs more than one candidate to walk.
        scored = []
        for candidate_idx in np.atleast_1d(nncandidates_idx):
            if candidate_idx >= self.nb_items:
                continue
            candidate_sc = self.scancontexts[candidate_idx]
            dist, _ = sc_utils.distance_sc(candidate_sc, query_sc)
            scored.append((dist, candidate_idx))

        if len(scored) == 0:
            return [None], [None]

        scored.sort(key=lambda candidate: candidate[0])
        best = scored[:max(k, 1)]
        return ([self.items[idx] for _, idx in best],
                [1.0 - dist for dist, _ in best])

    def search_best(self, query):
        """Search for the nearest neighbor
            Implementation for compatibily only

        Args:
            query (np.array): descriptor to match

        Returns:
            int, np.array: best match
        """
        if self.nb_items < 1:
            return None, None
            
        idxs, sims = self.search(query, 1)

        return idxs[0], sims[0]
