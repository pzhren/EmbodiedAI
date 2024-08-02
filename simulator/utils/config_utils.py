import numpy as np

MAIN_MODULE=[
    "sim",
    "scene",
    "tasks",
    "npcs",
    "scene"]
def merge_config(base_dict, extra_dict, inplace=True, verbose=False):
    """
    Iteratively updates @base_dict with values from @extra_dict.

    Args:
        base_dict (dict): Nested base dictionary, which should be updated with all values from @extra_dict
        extra_dict (dict): Nested extra dictionary, whose values will overwrite corresponding ones in @base_dict
        inplace (bool): Whether to modify @base_dict in place or not
        verbose (bool): If True, will print when keys are mismatched

    Returns:
        dict: Updated dictionary
    """
    base_dict = base_dict if inplace else deepcopy(base_dict)
    for k, v in extra_dict.items():
        if k not in base_dict:
            base_dict[k] = v
        else:
            if isinstance(v, dict) and isinstance(base_dict[k], dict):
                base_dict[k] = merge_config(base_dict[k], v)
            # if isinstance(v, list) and k in MAIN_MODULE:
            #     for i in range(len(v)):
            #         base_dict[k][i] = merge_config(base_dict[k][i], v)
            else:
                not_equal = base_dict[k] != v
                if isinstance(not_equal, np.ndarray):
                    not_equal = not_equal.any()
                if not_equal and verbose:
                    print(f"Different values for key {k}: {base_dict[k]}, {v}\n")
                # base_dict[k] = np.array(v) if isinstance(v, list) else v
                base_dict[k] = v

# Return new dict
    return base_dict