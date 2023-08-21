from ..vo import VelocityObstacle

class TestVelocityObstacle:
    def setup_method(self):
        rA = 0.5
        rB = 0.5
        self.vo = VelocityObstacle(radius_A=rA, radius_B=rB, time_horizon=1)

    def test_do_intersect(self):
        # cross
        P = [0,0]
        Q = [2,2]
        A = [0,2]
        B = [2,0]
        assert self.vo.do_intersect(P,Q,A,B) == True

        # on
        P = [0,0]
        Q = [2,2]
        A = [0,2]
        B = [1,1]
        assert self.vo.do_intersect(P,Q,A,B) == True

        # off
        P = [0,0]
        Q = [2,2]
        A = [1,1]
        B = [3,3]
        assert self.vo.do_intersect(P,Q,A,B) == False

    def test_in_triangle(self):
        # inside
        triangle = [[0,0],[1,0],[1,1]]
        point = [0.5, 0.25]
        assert self.vo.in_triangle(triangle, point) == True

        # on the vertex
        triangle = [[0,0],[1,0],[1,1]]
        point = [0.0, 0.0]
        assert self.vo.in_triangle(triangle, point) == False

        # on the edge
        triangle = [[0,0],[1,0],[1,1]]
        point = [0.5, 0.5]
        assert self.vo.in_triangle(triangle, point) == False

        # outside
        triangle = [[0,0],[1,0],[1,1]]
        point = [0, 1]
        assert self.vo.in_triangle(triangle, point) == False

    def test_tangent_points(self):
        import numpy as np
        O = [0, 0]
        P = [5, 5]
        r = 5
        t1, t2 = self.vo.tangent_points(O, r, P)
        assert np.linalg.norm(t1-[0, 5]) < 10e-10
        assert np.linalg.norm(t2 - [5, 0]) < 10e-10

