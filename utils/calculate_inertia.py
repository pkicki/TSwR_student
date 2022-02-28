import xml.etree.ElementTree as ET
import sys

if __name__ == "__main__":
    fname = sys.argv[1]
    urdf = ET.parse(fname)
    for link in urdf.findall('link'):
        m = -1.
        type = ''
        dims = {}
        inertial = link.find("inertial")
        if inertial:
            m = float(inertial.find("mass").attrib['value'])
        visual = link.find("visual")
        if visual:
            geometry = visual.find("geometry")
            if geometry:
                type = geometry[0].tag
                dims = {k: float(v) for k, v in geometry[0].attrib.items()}

        ixx, ixy, ixz, iyy, iyz, izz = 0., 0., 0., 0., 0., 0.
        if type == 'cylinder':
            r = dims['radius']
            h = dims['length']
            ixx = (m * (3 * r ** 2 + h ** 2)) / 12.
            iyy = ixx
            izz = (m * r ** 2) / 2.
        elif type == 'sphere':
            r = dims['radius']
            ixx = (2 * m * r ** 2) / 5.
            iyy = ixx
            izz = ixx
        if type:
            inertia = inertial.find("inertia")
            inertia.attrib = {k: f"{eval(k):.5f}" for k in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"]}
            a = 0
    urdf.write(sys.argv[1])

