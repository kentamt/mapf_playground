

class MPPIController:
    def __init__(self):

        self.p_u: None | list[tuple[float, float]] = None
        pass

    def compute_input(self)-> list[tuple[float, float]]:

        # C = [c1, ..., cK]]
        # U = [u1, u2, ..., uK]
        # for k in K:
        #     e = [e1, e2, ..., eT]
        #     u[k] = [u1, u2, ..., uT]
        #     Q = []
        #     fot t in [1, ..., T]:
        #         q = move(u[t]),
        #         Q.append(q)
        #     Q = [q1, q2, ..., qT]
        #     c[k] = cost(Q)
        #
        # W = [w1, w2, ..., wK]
        # U = [u1, u2, ..., uK]
        # u_out = w.dot(U')
        # return u_out
        pass