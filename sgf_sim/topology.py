"""Named graph topologies for numerical experiments."""

from __future__ import annotations

import numpy as np

from .config import IntArray

TOPOLOGY_NAMES = ("paper_fig1_reconstructed", "default", "ring", "complete")


def adjacency_for_topology(name: str, n: int) -> IntArray:
    """Return an undirected adjacency matrix for a named topology."""

    if name == "paper_fig1_reconstructed":
        return paper_fig1_reconstructed(n)
    if name == "default":
        return default_adjacency(n)
    if name == "ring":
        return ring_adjacency(n)
    if name == "complete":
        return complete_adjacency(n)
    raise ValueError(f"unknown topology: {name}")


def paper_fig1_reconstructed(n: int) -> IntArray:
    """Return a visual reconstruction of the paper's Fig. 1 graph.

    The paper publishes Fig. 1 as an image rather than a numeric edge list. This
    graph is reconstructed from the visible six-node simulation topology: an
    outer cycle plus two internal chords detected from the rendered figure.
    """

    if n != 6:
        raise ValueError("paper_fig1_reconstructed is only defined for n = 6")
    edges = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 4),
        (4, 5),
        (5, 0),
        (0, 2),
        (1, 4),
    ]
    return adjacency_from_edges(n, edges)


def default_adjacency(n: int) -> IntArray:
    """Return the earlier connected graph used by the first implementation."""

    edges = [(i, (i + 1) % n) for i in range(n)]
    if n >= 6:
        edges.extend([(0, 3), (1, 4), (2, 5)])
    return adjacency_from_edges(n, edges)


def ring_adjacency(n: int) -> IntArray:
    return adjacency_from_edges(n, [(i, (i + 1) % n) for i in range(n)])


def complete_adjacency(n: int) -> IntArray:
    adjacency = np.ones((n, n), dtype=int)
    np.fill_diagonal(adjacency, 0)
    return adjacency


def adjacency_from_edges(n: int, edges: list[tuple[int, int]]) -> IntArray:
    adjacency = np.zeros((n, n), dtype=int)
    for i, j in edges:
        adjacency[i, j] = 1
        adjacency[j, i] = 1
    return adjacency


def is_connected(adjacency: IntArray) -> bool:
    """Return True when all nodes are reachable in an undirected graph."""

    n = adjacency.shape[0]
    seen = {0}
    stack = [0]
    while stack:
        node = stack.pop()
        for neighbor in np.flatnonzero(adjacency[node]):
            if int(neighbor) not in seen:
                seen.add(int(neighbor))
                stack.append(int(neighbor))
    return len(seen) == n


def edge_list(adjacency: IntArray) -> list[tuple[int, int]]:
    """Return sorted undirected edges using zero-based robot indices."""

    edges: list[tuple[int, int]] = []
    for i in range(adjacency.shape[0]):
        for j in range(i + 1, adjacency.shape[1]):
            if adjacency[i, j]:
                edges.append((i, j))
    return edges
