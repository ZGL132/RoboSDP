import csv
import xml.etree.ElementTree as ET
import os

# ===== 路径配置 =====
base_dir = os.path.dirname(os.path.abspath(__file__))
csv_file = os.path.join(base_dir, "300kg.csv")
mesh_dir = os.path.join(base_dir, "meshes")

output_dir = os.path.join(base_dir, "output")
os.makedirs(output_dir, exist_ok=True)
output_file = os.path.join(output_dir, "300kg.urdf")

# ===== 创建 robot =====
robot = ET.Element("robot", name="300kg")

# ===== world 坐标（自动加）=====
ET.SubElement(robot, "link", name="world")

joint_world = ET.SubElement(robot, "joint", name="joint_to_world", type="fixed")
ET.SubElement(joint_world, "parent", link="world")
ET.SubElement(joint_world, "child", link="base")
ET.SubElement(joint_world, "origin", xyz="0 0 0", rpy="0 0 0") #两次旋转让Z轴向上，X轴向前，符合常见习惯。原本是y轴向前的。

# ===== 防止重复创建 link =====
created_links = set()

def safe(val, default="0"):
    return val if val not in ("", None) else default

def mesh_path(name):
    path = os.path.abspath(os.path.join(mesh_dir, f"{name}.STL"))
    return f"file:///{path}"

def create_link(row):
    name = row["Link Name"]
    if name in created_links:
        return
    created_links.add(name)

    link = ET.SubElement(robot, "link", name=name)

    # ===== inertial =====
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin",
        xyz=f'{safe(row["Center of Mass X"])} {safe(row["Center of Mass Y"])} {safe(row["Center of Mass Z"])}',
        rpy=f'{safe(row["Center of Mass Roll"])} {safe(row["Center of Mass Pitch"])} {safe(row["Center of Mass Yaw"])}'
    )

    ET.SubElement(inertial, "mass", value=safe(row["Mass"], "1"))

    ET.SubElement(inertial, "inertia",
        ixx=safe(row["Moment Ixx"]),
        ixy=safe(row["Moment Ixy"]),
        ixz=safe(row["Moment Ixz"]),
        iyy=safe(row["Moment Iyy"]),
        iyz=safe(row["Moment Iyz"]),
        izz=safe(row["Moment Izz"])
    )

    # ===== visual =====
    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin",
        xyz=f'{safe(row["Visual X"])} {safe(row["Visual Y"])} {safe(row["Visual Z"])}',
        rpy=f'{safe(row["Visual Roll"])} {safe(row["Visual Pitch"])} {safe(row["Visual Yaw"])}'
    )

    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "mesh", filename=mesh_path(name))

    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "color",
        rgba=f'{safe(row["Color Red"],"1")} {safe(row["Color Green"],"1")} {safe(row["Color Blue"],"1")} {safe(row["Color Alpha"],"1")}'
    )

    # ===== collision（默认用同一个mesh）=====
    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin",
        xyz=f'{safe(row["Collision X"])} {safe(row["Collision Y"])} {safe(row["Collision Z"])}',
        rpy=f'{safe(row["Collision Roll"])} {safe(row["Collision Pitch"])} {safe(row["Collision Yaw"])}'
    )

    geometry2 = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry2, "mesh", filename=mesh_path(name))


def create_joint(row):
    joint_name = row["Joint Name"]
    if joint_name == "":
        return

    parent = row["Parent"]
    child = row["Link Name"]

    joint_type = row["Joint Type"] if row["Joint Type"] else "fixed"

    joint = ET.SubElement(robot, "joint",
        name=joint_name,
        type=joint_type
    )

    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)

    ET.SubElement(joint, "origin",
        xyz=f'{safe(row["Joint Origin X"])} {safe(row["Joint Origin Y"])} {safe(row["Joint Origin Z"])}',
        rpy=f'{safe(row["Joint Origin Roll"])} {safe(row["Joint Origin Pitch"])} {safe(row["Joint Origin Yaw"])}'
    )

    # ===== axis（有才写）=====
    ax = row["Joint Axis X"]
    ay = row["Joint Axis Y"]
    az = row["Joint Axis Z"]

    if ax and ay and az:
        ET.SubElement(joint, "axis", xyz=f"{ax} {ay} {az}")

    # ===== limit（自动补全）=====
    effort = safe(row["Limit Effort"], "10")
    velocity = safe(row["Limit Velocity"], "1")

    if joint_type == "continuous":
        ET.SubElement(joint, "limit",
            effort=effort,
            velocity=velocity
        )
    else:
        ET.SubElement(joint, "limit",
            effort=effort,
            velocity=velocity,
            lower=safe(row["Limit Lower"], "-3.14"),
            upper=safe(row["Limit Upper"], "3.14")
        )

    # ===== dynamics（有才写）=====
    damping = row["Dynamics Damping"]
    friction = row["Dynamics Friction"]

    if damping or friction:
        ET.SubElement(joint, "dynamics",
            damping=safe(damping),
            friction=safe(friction)
        )


# ===== 读取 CSV =====
with open(csv_file, newline='', encoding='gbk') as f:
    reader = csv.DictReader(f)
    rows = list(reader)

    for row in rows:
        create_link(row)

    for row in rows:
        create_joint(row)

# ===== 美化输出 =====
tree = ET.ElementTree(robot)
ET.indent(tree, space="  ")

tree.write(output_file, encoding="utf-8", xml_declaration=True)

print("✅ URDF生成成功：", output_file)