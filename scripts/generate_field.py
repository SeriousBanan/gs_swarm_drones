"""
Module that generate graph of some parallelogram like field
and write it as json to file with name `SAVE_TO_FILE_NAME` for further
reading and initializing.

First write info about vertexes. Then write info about edges.

Output file content:
{
    "vertexes": [
        {
            "id": 1,
            "x": 0,
            "y": 0,
        },
        {
            "id": 2,
            "x": 0,
            "y": 1,
        },
        {
            "id": 3,
            "x": 1,
            "y": 0,
        },
    ],
    "edges": [
        {
            "from id": 1,
            "to id": 2,
        },
        {
            "from id": 1,
            "to id": 3,
        },
        {
            "from id": 2,
            "to id": 1,
        },
        {
            "from id": 2,
            "to id": 3,
        },
        {
            "from id": 3,
            "to id": 1,
        },
        {
            "from id": 3,
            "to id": 2,
        }
    ]
}

which initialize next graph:
(1) - (2)
 |  /
(3)
"""

import argparse
import json
from typing import Any, Dict, List, Tuple


def save_graph_to_file(size: Tuple[float, float],
                       vertex_size: Tuple[float, float],
                       file_name: str) -> None:
    """Generate graph of size: `size` and save it as json to file named `file_name`."""

    data: Dict[str, List[Dict[str, Any]]] = {
        "vertexes": [],
        "edges": [],
    }

    max_x = int(size[0] // vertex_size[0])
    max_y = int(size[1] // vertex_size[1])

    for x in range(max_x):
        for y in range(max_y):
            data["vertexes"].append({
                "id": x * max_x + y,
                "x": x * vertex_size[0],
                "y": y * vertex_size[1],
            })

            for dx, dy in [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]:
                if 0 <= x + dx < max_x and 0 <= y + dy < max_y:
                    data["edges"].append({
                        "from id": x * max_x + y,
                        "to id": (x + dx) * max_x + (y + dy),
                    })

    with open(file_name, "w") as file:
        json.dump(data, file, indent=4)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate graph and save it to file.")
    parser.add_argument("width", type=float, help="Field width.")
    parser.add_argument("height", type=float, help="Field height.")
    parser.add_argument("vertex_width", type=float, help="Vertex height.")
    parser.add_argument("vertex_height", type=float, help="Vertex height.")
    parser.add_argument("--filename",
                        default="field.json",
                        help="Name of file where to save field. Default: field.json")

    args = parser.parse_args()

    save_graph_to_file((args.width, args.height),
                       (args.vertex_width, args.vertex_height),
                       args.filename)
