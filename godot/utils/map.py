from typing import Dict, List, Set, Tuple

import godot_parser
from godot_parser import GDScene

import launcher.config


class Node:
  def __init__(self, id: int, cord: Tuple[int, int]):
    self.id = id
    self.x = cord[0]
    self.y = cord[1]

  def __str__(self) -> str:
    return f"Node(id={self.id}, x={self.x}, y={self.y})"

  def __repr__(self) -> str:
    return self.__str__()


def _get_cord_from_tile(tile) -> Tuple[int, int]:
  """
  Assuming map is made with map_maker.tscn and tiles are named "NAME_X_Y"
  """
  name = tile.name
  grid_num = name.split('_')
  return int(grid_num[1]), int(grid_num[2])


def get_nodes(godot_scene_name) -> list[Node] | None:
  file_name = godot_scene_name + ".tscn"

  path = launcher.config.GODOT_MAP_DIR / file_name
  scene: GDScene = godot_parser.load(path)
  with scene.use_tree() as tree:
    tiles = tree.get_node("Tiles")
    if tiles is not None:
      return [Node(i, _get_cord_from_tile(tile)) for i, tile in
              enumerate(tiles.get_children())]
    else:
      print("Loaded scene without 'Tiles' node")
      return None


if __name__ == "__main__":
  print(get_nodes("test"))