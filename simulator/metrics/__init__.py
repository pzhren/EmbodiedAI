from simulator.core.register import registry

def make_metric(id_metric: str, *args):
    metric = registry.get_metric(id_metric)
    assert metric is not None, "Could not find metric with name {}".format(
        id_metric)
    return metric(*args)