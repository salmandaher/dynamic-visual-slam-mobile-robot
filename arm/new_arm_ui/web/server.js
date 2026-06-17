#!/usr/bin/env node
// new_arm web server -- a ROS 2 node (via rclnodejs) that also serves the web UI.
//
// The Node process IS a ROS 2 node: no rosbridge needed. It:
//   * serves the browser app (index.html / app.js / fk.js / style.css) over HTTP,
//   * bridges the browser <-> ROS over a WebSocket,
//   * Cartesian goals  -> /compute_ik (MoveIt) -> FollowJointTrajectory action,
//   * named poses      -> FollowJointTrajectory action,
//   * raw joint jogs   -> /arm/joint_command (std_msgs/Float32MultiArray),
//   * /joint_states    -> pushed to all browsers for the live schematic.
//
// REQUIRES Node 18 or 20 LTS (rclnodejs's native addon does NOT build on Node 25).
// Setup:  nvm install 20 && nvm use 20 && (in this dir) npm install
// Run:    node server.js          (with a ROS 2 env sourced)
// Env:    PORT (default 8080)

const path = require('path');
const http = require('http');
const express = require('express');
const { WebSocketServer } = require('ws');
const rclnodejs = require('rclnodejs');

const PORT = parseInt(process.env.PORT || '8080', 10);
const ACTUATED = ['after_base_full_joint', 'link1_joint', 'link2_joint'];
const NAMED = { home: [0.0, 0.0, 0.0], ready: [0.0, 0.4, -0.6] };
const GROUP = 'new_arm';
const IK_LINK = 'end_effector_link';
const PLANNING_FRAME = 'world';
const ACTION = '/new_arm_controller/follow_joint_trajectory';

function durationMsg(seconds) {
  return { sec: Math.floor(seconds), nanosec: Math.floor((seconds % 1) * 1e9) };
}

async function main() {
  await rclnodejs.init();
  const node = new rclnodejs.Node('new_arm_web');

  const jogPub = node.createPublisher('std_msgs/msg/Float32MultiArray', '/arm/joint_command');
  const ikClient = node.createClient('moveit_msgs/srv/GetPositionIK', '/compute_ik');
  const trajClient = new rclnodejs.ActionClient(
    node, 'control_msgs/action/FollowJointTrajectory', ACTION);

  let lastJointState = null;
  let currentGoal = null; // for Stop/cancel

  node.createSubscription('sensor_msgs/msg/JointState', '/joint_states', (msg) => {
    lastJointState = { name: Array.from(msg.name), position: Array.from(msg.position) };
    broadcast({ type: 'joint_states', name: lastJointState.name, position: lastJointState.position });
  });

  rclnodejs.spin(node);

  // --- HTTP + WebSocket ---
  const app = express();
  app.use(express.static(__dirname));
  const server = http.createServer(app);
  const wss = new WebSocketServer({ server });

  function broadcast(obj) {
    const s = JSON.stringify(obj);
    wss.clients.forEach((c) => { if (c.readyState === 1) c.send(s); });
  }
  function status(text) {
    console.log('[status]', text);
    broadcast({ type: 'status', text });
  }

  wss.on('connection', (ws) => {
    ws.send(JSON.stringify({ type: 'status', text: 'connected to new_arm_web' }));
    ws.send(JSON.stringify({ type: 'config', actuated: ACTUATED, named: Object.keys(NAMED) }));
    if (lastJointState) {
      ws.send(JSON.stringify({ type: 'joint_states', ...lastJointState }));
    }
    ws.on('message', (data) => {
      let cmd;
      try { cmd = JSON.parse(data); } catch (e) { return; }
      handle(cmd).catch((err) => status('error: ' + err.message));
    });
  });

  async function handle(cmd) {
    if (cmd.type === 'jog') {
      const t = (cmd.time || 1.0) * 1000.0;
      const msg = { data: ACTUATED.map((_, i) => Number(cmd.joints[i] || 0)).concat([t]) };
      jogPub.publish(msg);
      status(`jog sent: [${cmd.joints.map((v) => (+v).toFixed(3)).join(', ')}]`);
    } else if (cmd.type === 'named') {
      if (!NAMED[cmd.name]) { status(`unknown named pose '${cmd.name}'`); return; }
      status(`moving to named '${cmd.name}'...`);
      await execute(NAMED[cmd.name], cmd.time || 2.0, `named '${cmd.name}'`);
    } else if (cmd.type === 'goto') {
      status(`planning to XYZ (${(+cmd.x).toFixed(3)}, ${(+cmd.y).toFixed(3)}, ${(+cmd.z).toFixed(3)})...`);
      const joints = await solveIk(cmd.x, cmd.y, cmd.z);
      if (joints) await execute(joints, cmd.time || 2.0, 'Cartesian goal');
    } else if (cmd.type === 'stop') {
      if (currentGoal) {
        try { await currentGoal.cancelGoal(); status('stop: goal cancelled.'); }
        catch (e) { status('stop: ' + e.message); }
      } else {
        status('stop: nothing executing.');
      }
    }
  }

  async function solveIk(x, y, z) {
    const ready = await ikClient.waitForService(4000);
    if (!ready) { status('/compute_ik unavailable -- is move_group running?'); return null; }
    const request = {
      ik_request: {
        group_name: GROUP,
        ik_link_name: IK_LINK,
        avoid_collisions: true,
        timeout: durationMsg(2.0),
        pose_stamped: {
          header: { frame_id: PLANNING_FRAME },
          pose: {
            position: { x: Number(x), y: Number(y), z: Number(z) },
            orientation: { x: 0, y: 0, z: 0, w: 1 },
          },
        },
      },
    };
    const response = await new Promise((resolve) => {
      ikClient.sendRequest(request, (resp) => resolve(resp));
    });
    if (!response) { status('IK call failed.'); return null; }
    if (response.error_code.val !== 1) {
      status(`no IK solution (code ${response.error_code.val}); target likely out of reach.`);
      return null;
    }
    const js = response.solution.joint_state;
    const map = {};
    for (let i = 0; i < js.name.length; i++) map[js.name[i]] = js.position[i];
    const joints = ACTUATED.map((j) => map[j]);
    if (joints.some((v) => v == null)) { status('IK solution missing a joint.'); return null; }
    return joints;
  }

  async function execute(joints, moveTime, label) {
    const ready = await trajClient.waitForServer(4000);
    if (!ready) { status('controller action unavailable -- is arm_bridge running?'); return; }
    const goal = {
      trajectory: {
        joint_names: ACTUATED,
        points: [{
          positions: joints.map(Number),
          time_from_start: durationMsg(moveTime),
        }],
      },
    };
    const goalHandle = await trajClient.sendGoal(goal);
    if (!goalHandle.isAccepted()) { status('trajectory goal rejected.'); return; }
    currentGoal = goalHandle;
    status(`executing ${label} over ${moveTime.toFixed(1)}s...`);
    const result = await goalHandle.getResult();
    currentGoal = null;
    const code = result && result.error_code != null ? result.error_code : -1;
    if (code === 0) status(`done: reached ${label}.`);
    else status(`execution finished with error_code ${code}.`);
  }

  server.listen(PORT, () => {
    console.log(`new_arm_web UI on http://localhost:${PORT}`);
    status(`web server listening on :${PORT}`);
  });

  process.on('SIGINT', () => { rclnodejs.shutdown(); process.exit(0); });
}

main().catch((err) => { console.error(err); process.exit(1); });
