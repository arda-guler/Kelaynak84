import numpy as np

class Model:
    def __init__(self, filepath):
        self.filepath = self.sanitize_filepath(filepath)
        self.vertices, self.lines, self.faces = self.read_model()

    def sanitize_filepath(self, path):
        if not path.endswith(".mdl"):
            path = path + ".mdl"

        if not path.startswith("data/models/"):
            path = "data/models/" + path

        return path

    def read_model(self):
        mfile = open(self.filepath, "r")
        mlines = mfile.readlines()

        vertices = []
        lines = []
        faces = []
        for l in mlines:
            # vertices
            if l.startswith("V|"):
                # get rid of 'V|'
                l = l[2:]
                # get rid of spaces
                l = l.replace(" ", "")
                l = l.replace("\n", "")
                l = l.split(",")
                
                vertices.append(np.array([float(l[0]),
                                          float(l[1]),
                                          float(l[2])]))

            # lines
            elif l.startswith("L|"):
                l = l[2:]
                l = l.replace(" ", "")
                l = l.replace("\n", "")
                l = l.split(",")

                lines.append([int(l[0]) - 1, int(l[1]) - 1])

            # faces
            elif l.startswith("F|"):
                l = l[2:]
                l = l.replace(" ", "")
                l = l.replace("\n", "")
                l = l.split(",")

                new_face = []
                for i in l:
                    new_face.append(int(i) - 1)
                    
                faces.append(new_face)

        return vertices, lines, faces
