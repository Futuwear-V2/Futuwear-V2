 "use strict"

b4w.register("torso", function(exports, require) {

	const m_app    = require("app");
	const m_data   = require("data");
	const m_scs    = require("scenes");
	const m_armat  = require("armature");
	const m_tsr    = require("tsr");
	const m_quat	 = require("quat");
	const m_geom	 = require("geometry");
    const m_obj	 = require("objects");
	
    const bones = {
        Back_Lower: {
            X: { Rot: 0, Min: -90, Max: 90 },
            Y: { Rot: 0, Min: -90, Max: 90 },
            Z: { Rot: 0, Min: -90, Max: 90 },
        },
        Back_Middle: {
            X: { Rot: 0, Min: -90, Max: 90 },
            Y: { Rot: 0, Min: -90, Max: 90 },
            Z: { Rot: 0, Min: -90, Max: 90 },
        },
        Back_Upper: {
            X: { Rot: 0, Min: -90, Max: 90 },
            Y: { Rot: 0, Min: -90, Max: 90 },
            Z: { Rot: 0, Min: -90, Max: 90 },
        },
        Neck: {
            X: { Rot: 0, Min: -90, Max: 90 },
            Y: { Rot: 0, Min: -90, Max: 90 },
            Z: { Rot: 0, Min: -90, Max: 90 },
        },
        R_Shoulder: {
            X: { Rot: 0, Min: -90, Max: 90 },
            Y: { Rot: 0, Min: -90, Max: 90 },
            Z: { Rot: 0, Min: -90, Max: 90 },
        },
        R_Arm_Inner: {
            X: { Rot: 0, Min: -90, Max: 90 },
            Y: { Rot: 0, Min: -90, Max: 90 },
            Z: { Rot: 0, Min: -90, Max: 90 },
        },
        R_Arm_Outer: {
            X: { Rot: 0, Min: -90, Max: 90 },
            Y: { Rot: 0, Min: -90, Max: 90 },
            Z: { Rot: 0, Min: -90, Max: 90 },
        },
        L_Shoulder: {
            X: { Rot: 0, Min: -90, Max: 90 },
            Y: { Rot: 0, Min: -90, Max: 90 },
            Z: { Rot: 0, Min: -90, Max: 90 },
        },
        L_Arm_Inner: {
            X: { Rot: 0, Min: -90, Max: 90 },
            Y: { Rot: 0, Min: -90, Max: 90 },
            Z: { Rot: 0, Min: -90, Max: 90 },
        },
        L_Arm_Outer: {
            X: { Rot: 0, Min: -90, Max: 90 },
            Y: { Rot: 0, Min: -90, Max: 90 },
            Z: { Rot: 0, Min: -90, Max: 90 },
        },
    };

	exports.init = function() {
		m_app.init({
			canvas_container_id: "canvas_cont",
			callback: init_cb,
			show_fps: false,
			autoresize: true,
			console_verbose: false
		});
	}

	function init_cb(canvas_elem, success) {
		if (!success) {
			console.log("b4w init failure");
			return;
		}
		load();
	}

	function load() {
		m_data.load("torso.json", load_cb);
	}

	function load_cb(data_id) {
		m_app.enable_camera_controls();
	}

	exports.rotate_all = function () {

		Object.keys(bones).forEach( (bone_name, index) => {
			var bone = bones[bone_name];
            rotate_bone(bone_name, bone.X.Rot, bone.Y.Rot, bone.Z.Rot);
		});
	}
		
	function euler_to_quat(roll, pitch, yaw) {
		//Converts rotation coordinates from euer to quaternion, which is used by tsr transform.
		var cr, cp, cy, sr, sp, sy, cpcy, spsy, w, x, y, z;
		cr = Math.cos(roll/2);
		cp = Math.cos(pitch/2);
		cy = Math.cos(yaw/2);
		sr = Math.sin(roll/2);
		sp = Math.sin(pitch/2);
		sy = Math.sin(yaw/2);
		cpcy = cp * cy;
		spsy = sp * sy;
		
		w = cr * cpcy + sr * spsy;
		x = sr * cpcy - cr * spsy;
		y = cr * sp * cy + sr * cp * sy;
		z = cr * cp * sy - sr * sp * cy;
		var quat = [x, y, z, w];
		return quat;
	}
		
	function dtr(val) {
		//converts degrees to radians
		var rads = val*Math.PI/180;
		return rads;
		
	}

	function constrain(val, min, max) {
		// Constrain value between minimum and maximum values
		return Math.min(Math.max(val, min), max);
	}

	function rotate_bone(bone_name, x, y, z) {
		var rig = m_scs.get_object_by_name("Armature");

		var arm_tsr = [0, 0, 0, 1];
		var quat = euler_to_quat(x, y, z);
		m_tsr.set_quat(quat, arm_tsr);
		//tsr format is: [Tx,Ty,Tz,S,Rx,Ry,Rz,Rw];
		m_armat.set_bone_tsr_rel(rig, bone_name, arm_tsr);
	}
		
	exports.sensor_update_degree = function (bone_name, axis_name, degree) {
        var bone = bones[bone_name];
		if (!bone) return false;
		
		var axis = bone[axis_name]
		if (!axis) return false;
		
		// Set new rotation within constraints
        axis.Rot = dtr(constrain(degree, axis.Min, axis.Max));
		
		return true;
	}

});

var animator = b4w.require("torso"); 
animator.init();