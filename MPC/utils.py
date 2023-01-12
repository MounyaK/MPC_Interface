import sys


def centroid(vertexes:list):
        try:
            _x_list = [vertex [0] for vertex in vertexes]
            _y_list = [vertex [1] for vertex in vertexes]
            _len = len(vertexes)
            _x = sum(_x_list) / _len
            _y = sum(_y_list) / _len
        
        except:
            e = str(sys.exc_info()[0]) + ": " + str(sys.exc_info()[1])
            raise Exception( "MPC.simulator.centroid():\n"+ str(e) )
        
        return [_x, _y]