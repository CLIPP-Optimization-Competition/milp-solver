"""
Objective-function evaluation for CLIPP solutions.
"""

__all__ = ["compute_score"]


def compute_score(
    instance: dict,
    clean_per_vehicle: list[list[int]],
) -> tuple[float, int, float, set[int]]:
    """
    Compute the CLIPP objective value for a given solution.

    Parameters
    ----------
    instance : dict
        Parsed instance.
    clean_per_vehicle : list[list[int]]
        Per-vehicle lists of cleaned street indices.

    Returns
    -------
    score : float
        ``total_length - W * total_waste``.
    total_length : int
        Sum of cleaned street lengths (metres).
    total_waste : float
        Total water wasted (litres).
    cleaned_set : set[int]
        Set of cleaned street indices.
    """
    streets  = instance["streets"]
    vehicles = instance["vehicles"]
    W        = instance["W"]

    cleaned_length: dict[int, int] = {}  # j → length (counted once)
    total_waste = 0.0

    for k, cleaned in enumerate(clean_per_vehicle):
        for j in cleaned:
            s = streets[j]
            if s["cat"] == "C":
                continue
            waste = (vehicles[k]["cap"] - s["req"]) * s["length"] / 1000.0
            total_waste += waste
            if j not in cleaned_length:
                cleaned_length[j] = s["length"]

    total_length = sum(cleaned_length.values())
    score        = total_length - W * total_waste
    return score, total_length, total_waste, set(cleaned_length.keys())
