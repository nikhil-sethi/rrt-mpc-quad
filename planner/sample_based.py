from spaces import Space
from graph import Graph

class SamplingPlanner:
    def __init__(self, start, goal, space:Space, map) -> None:
        self.start = start
        self.goal = goal
        self.map:list = map # list of obstacles
        self.space = space
        self.graph = Graph(init_node=start) 
    
    def get_free_sample(self):
        colliding = True
        while colliding:
            sample = self.space.sample()
            for obs in self.map:
                if obs.is_colliding(sample):
                    colliding = True
                else:
                    colliding = False
        return sample
    
    def get_free_connection(self, q, q_p):
        return (q,q_p)

class RRT(SamplingPlanner):
    def __init__(self, start, goal, space:Space, map) -> None:
        super().__init__(self, start, goal, space, map)
        self.best_cost = 0
        self.max_depth = 10

    def step(self):
        """one step of the planner"""
        
        # Sample a point in free space
        q = self.get_free_sample()
        q_p, cost = self.graph.closest_node(q)
        e = self.get_free_connection(q, q_p)
        
        return q, e, cost
        """
        initialise graph with start node
        sample a new point
        check for collision with obstacles in map
        do until point in free space is found
        """
    def backtrack(self, point):
        """Returns a continuous connected path from point to start node"""
        

    def run(self):
        for i in range(self.max_depth):
            q, e, cost = self.step()
            self.graph.add_edge(e)
            self.graph.add_node(q)

            dist_to_target = self.graph.euclidean_metric(q, self.goal)
            if cost < dist_to_target:
                return self.backtrack()