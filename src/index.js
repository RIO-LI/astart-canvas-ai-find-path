import "./styles.css";

import Konva from "konva";
import { cloneDeep } from "lodash";

const width = window.innerWidth;
const height = 1500;

class AStarNode {
  constructor({ x, y, parent, end, type = AStarNode.NODE_TYPE.OPEN }) {
    this.x = x;
    this.y = y;
    this.parent = parent != null ? parent : null;
    this.end = end != null ? end : this;
    this.type = type;
    this._g = 0;
  }

  // 代价
  get f() {
    return this.h + this.g;
  }

  // 距离起点的距离
  get g() {
    return this._g;
  }

  set g(val) {
    this._g = val !== null ? val : 0;
  }

  // 距离终点的曼哈顿距离
  get h() {
    if (this.end === this) {
      return 0;
    }
    return this.getManhattanDistance(this, this.end);
  }

  // 获取两个节点之间的曼哈顿距离
  getManhattanDistance(source, target) {
    return Math.abs(source.x - target.x) + Math.abs(source.y - target.y);
  }

  // 获取锚点元素的四个方向的邻居元素的网格坐标
  // 上右下左
  getNeighbors() {
    const { x: ax, y: ay } = this;
    return [
      new AStarNode({ x: ax, y: ay - 1, parent: this, end: this.end }),
      new AStarNode({ x: ax + 1, y: ay, parent: this, end: this.end }),
      new AStarNode({ x: ax, y: ay + 1, parent: this, end: this.end }),
      new AStarNode({ x: ax - 1, y: ay, parent: this, end: this.end })
    ];
  }

  isEqualTo(o) {
    return this.x === o.x && this.y === o.y;
  }
}

AStarNode.NODE_TYPE = {
  OPEN: 1,
  CLOSE: 2
};

class AStarMgr {
  constructor({
    sourceShape,
    sourceAnchorDirection,
    targetShape,
    targetAnchorDirection,
    mapWidth,
    mapHeight,
    obstacles = [],
    anchorOffset = 10,
    step = 10
  }) {
    this.anchorOffset = anchorOffset;
    this.step = step;
    this.sourceShape = sourceShape;
    this.targetShape = targetShape;

    this.oSource = this.getAnchor(sourceShape, sourceAnchorDirection, 0);
    this.oTarget = this.getAnchor(targetShape, targetAnchorDirection, 0);
    this.source = this.getAnchor(
      sourceShape,
      sourceAnchorDirection,
      this.anchorOffset
    );
    this.target = this.getAnchor(
      targetShape,
      targetAnchorDirection,
      this.anchorOffset
    );
    this.grid = this.getGrid(
      this.source,
      this.target,
      mapWidth,
      mapHeight,
      this.step
    );
    this.sourceCell = this.convertToGridCell(this.source, this.grid);
    this.targetCell = this.convertToGridCell(this.target, this.grid);

    this.obstacles = this.convertObstaclesToGridCell(
      [this.sourceShape, ...obstacles, this.targetShape],
      this.anchorOffset
    );

    this.openList = [];
    this.closeList = [];
  }

  findPath(limit = 2000) {
    let hasFound = false;
    this.openList = [];
    this.closeList = [];
    const start = new AStarNode({
      x: this.sourceCell.x,
      y: this.sourceCell.y,
      end: this.targetCell,
      parent: null
    });
    const end = new AStarNode({
      x: this.targetCell.x,
      y: this.targetCell.y,
      end: this.targetCell
    });
    this.openList = this.sortNeighbors(
      this.getNeighbors(start, this.closeList, this.openList, this.grid)
    );
    this.closeList.push(start);
    let count = 0;
    while (!hasFound && this.openList.length > 0 && count <= limit) {
      count++;
      const cell = this.openList.shift();
      this.closeList.push(cell);
      if (cell.isEqualTo(end)) {
        hasFound = true;
        break;
      }
      this.openList = this.sortNeighbors(
        this.openList.concat(
          this.getNeighbors(cell, this.closeList, this.openList, this.grid)
        )
      );
    }
    return this.getPath(
      cloneDeep(this.closeList.reverse()),
      this.grid,
      this.oSource,
      this.oTarget
    );
  }

  getPath(closeList, grid, source, target) {
    let curr = closeList.shift();
    let path = [curr];
    while (curr.parent) {
      for (let i = 0, len = closeList.length; i < len; i++) {
        const cl = closeList[i];
        if (cl.isEqualTo(curr.parent)) {
          path.push(cl);
          closeList.splice(i, 1);
          curr = cl;
          break;
        }
      }
    }
    path = path.reverse().map((p) => {
      return {
        x: p.x * grid.x + grid.source.x,
        y: p.y * grid.y + grid.source.y
      };
    });
    path.unshift(source);
    path.push(target);
    return path;
  }

  getNeighbors(cell, closeList, openList, grid) {
    return cell
      .getNeighbors()
      .filter((nb) => {
        return (
          !this.obstacles.some((obc) => this.rectContainsPoint(obc, nb)) &&
          !closeList.some((cl) => cl.isEqualTo(nb)) &&
          !openList.some((ol) => ol.isEqualTo(nb)) &&
          nb.x >= grid.edge.left &&
          nb.x <= grid.edge.right &&
          nb.y >= grid.edge.top &&
          nb.y <= grid.edge.bottom
        );
      })
      .map((nb) => {
        nb.g = nb.parent.g + 1;
        return nb;
      });
  }

  sortNeighbors(neighbors) {
    return neighbors.sort((a, b) => a.f - b.f);
  }

  // 查找指定方位的锚点
  getAnchor(shape, anchorDirection, anchorOffset) {
    const { x, y, width, height } = shape;
    const anchor = { x, y };
    switch (anchorDirection) {
      case "top":
        anchor.x += width / 2;
        anchor.y -= anchorOffset;
        break;
      case "right":
        anchor.x += width + anchorOffset;
        anchor.y += height / 2;
        break;
      case "bottom":
        anchor.x += width / 2;
        anchor.y += height + anchorOffset;
        break;
      case "left":
        anchor.x -= anchorOffset;
        anchor.y += height / 2;
        break;
    }
    return anchor;
  }

  // 获取两个节点之间的欧几里得距离
  getEuclideanDistance(source, target) {
    return Math.sqrt((source.x - target.x) ** 2 + (source.y - target.y) ** 2);
  }

  // 将坐标点转化为格子坐标
  convertToGridCell(point, grid) {
    return {
      x: Math.floor((point.x - this.source.x) / grid.x),
      y: Math.floor((point.y - this.source.y) / grid.y)
    };
  }

  // 将障碍物的坐标系专为格子坐标
  convertObstaclesToGridCell(obstacles, padding) {
    return obstacles.map((obstacle, index, arr) => {
      const { x, y, width, height } = obstacle;
      const cell = this.convertToGridCell(
        {
          x: x - padding,
          y: y - padding
        },
        this.grid
      );
      cell.width = Math.ceil((width + 2 * padding) / this.grid.x);
      cell.height = Math.ceil((height + 2 * padding) / this.grid.y);
      return cell;
    });
  }

  // 获取原始锚点和目标锚点之间根据探索步伐所形成网格信息，即每一个格的宽度和高度
  getGrid(source, target, mapWidth, mapHeight, step) {
    const dx = this.getGridDimension(target.x - source.x, step);
    const dy = this.getGridDimension(target.y - source.y, step);
    return {
      source: cloneDeep(source),
      x: dx,
      y: dy,
      // edge用来约束寻路地图的边界
      edge: {
        // 进行+1是为了贴近边缘的元素也算在有效地图之中
        top: Math.ceil(-source.y / dy) - 1,
        right: Math.ceil((mapWidth - source.x) / dx) + 1,
        left: Math.ceil(-source.x / dx) - 1,
        bottom: Math.ceil((mapHeight - source.y) / dy) + 1
      }
    };
  }

  // 获取实际上的网格步伐大小，因为原始锚点和目标锚点之间的曼哈顿距离不一定可以被步伐大小所整除，需要作出调整
  getGridDimension(diff, step) {
    // return step if diff = 0
    if (!diff) {
      return step;
    }
    const abs = Math.abs(diff);
    const count = Math.round(abs / step);
    // return `abs` if less than one step apart
    if (!count) {
      return abs;
    }
    // otherwise, return corrected step
    const roundedDiff = count * step;
    const remainder = abs - roundedDiff;
    const correction = remainder / count;
    return step + correction;
  }

  // 判断一个点是否在矩形区域内
  rectContainsPoint(rect, point) {
    const { x, y } = point;
    const { x: rx, y: ry } = rect;
    const { x: crx, y: cry } = {
      x: rect.x + rect.width,
      y: rect.y + rect.height
    };
    // 全部改为开区间，这样遇到起始节点所在图形或者结束节点所在图形，可以步行到起始或者结束节点，完成连接
    const res = x > rx && x < crx && y > ry && y < cry;
    return res;
  }
}

const stage = new Konva.Stage({
  container: "app",
  width,
  height
});

const layer = new Konva.Layer();
stage.add(layer);

const box1 = new Konva.Rect({
  x: 40,
  y: 150,
  width: 100,
  height: 50,
  fill: "#00D2FF",
  stroke: "black",
  draggable: true
});

const box2 = new Konva.Rect({
  x: 300,
  y: 100,
  width: 300,
  height: 300,
  fill: "yellow",
  stroke: "black",
  draggable: true
});

const box3 = new Konva.Rect({
  x: 800,
  y: 300,
  width: 200,
  height: 200,
  fill: "red",
  stroke: "black",
  draggable: true
});

const box4 = new Konva.Rect({
  x: 600,
  y: 10,
  width: 100,
  height: 100,
  fill: "green",
  stroke: "pink",
  draggable: true
});

const box5 = new Konva.Rect({
  x: 600,
  y: 400,
  width: 150,
  height: 700,
  fill: "green",
  stroke: "pink",
  draggable: true
});

const arrow = new Konva.Arrow({
  x: 0,
  y: 0,
  points: [],
  pointerLength: 20,
  pointerWidth: 20,
  fill: "black",
  stroke: "black",
  strokeWidth: 1.5
});

layer.add(box1, box2, box3, box4, box5, arrow);

console.time("findPath");
const path = new AStarMgr({
  sourceShape: box1.attrs, // 起始节点的形状，包括坐标和宽高
  sourceAnchorDirection: "bottom", // 起始节点连接的锚点方位
  targetShape: box3.attrs, // 结束节点的形状，包括坐标和宽高
  targetAnchorDirection: "bottom", // 结束节点连接的锚点方位
  obstacles: [box2.attrs, box4.attrs, box5.attrs], // 障碍物，路径会绕过他，无法绕过时，会穿过他们
  anchorOffset: 30, // 连接锚点离元素的距离
  step: 27, // 寻路每一次前进的步伐
  mapWidth: width, // 寻路地图的宽度
  mapHeight: height // 寻路地图的高度
}).findPath();
console.timeEnd("findPath");

arrow.setAttrs({
  points: path.map((item) => [item.x, item.y]).flat(Infinity)
});
