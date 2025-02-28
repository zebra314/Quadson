import xml.etree.ElementTree as ET

def add_collision_to_urdf(urdf_path, output_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    for link in root.findall("link"):
        visual = link.find("visual")
        if visual is not None:
            # Copy origin and geometry from visual
            origin = visual.find("origin")
            geometry = visual.find("geometry")

            if geometry is not None:
                mesh = geometry.find("mesh")
                if mesh is not None:
                    # Create a new collision element
                    collision = ET.Element("collision")
                    collision.set("name", link.get("name") + "_collision")

                    # Copy origin
                    if origin is not None:
                        collision.append(origin)

                    # Copy geometry (mesh)
                    new_geometry = ET.Element("geometry")
                    new_mesh = ET.Element("mesh")
                    new_mesh.set("filename", mesh.get("filename"))
                    new_mesh.set("scale", mesh.get("scale", "1.00000 1.00000 1.00000"))
                    new_geometry.append(new_mesh)

                    collision.append(new_geometry)

                    # Append collision object to the link
                    link.append(collision)

    # Save modified URDF
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    print(f"Modified URDF saved to {output_path}")

# Usage
input_urdf = "./urdf/quadson.urdf"
output_urdf = "./urdf/quadson_modified.urdf"
add_collision_to_urdf(input_urdf, output_urdf)
