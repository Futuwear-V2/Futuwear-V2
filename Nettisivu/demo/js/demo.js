var bones = {
    "L_Arm_Outer" : {
        X: {
            angle: 0,
            direction: 1,
        },
        Y: {
            angle: 0,
            direction: 1,
        },
    },
    "L_Arm_Inner" : {
        X: {
            angle: 0,
            direction: 1,
        },
        Y: {
            angle: 0,
            direction: 1,
        },
    },
    "R_Arm_Outer" : {
        X: {
            angle: 0,
            direction: 1,
        },
        Y: {
            angle: 0,
            direction: 1,
        },
    },
    "R_Arm_Inner" : {
        X: {
            angle: 0,
            direction: 1,
        },
        Y: {
            angle: 0,
            direction: 1,
        },
    },
}

function sendNewAngle()
{
    Object.keys(bones).forEach((bone_name, i) => {
        var bone = bones[bone_name];
        Object.keys(bone).forEach((axis_name, i) => {
            var axis = bone[axis_name];
            var rand = Math.random() * 180 - 90;
            if (rand > axis.angle + 120 || axis.angle <= -90)
                axis.direction = 1;
            else if (rand < axis.angle - 120 || axis.angle >= 90)
                axis.direction = -1; 
            axis.angle = Math.min(Math.max(axis.angle + axis.direction, -90), 90);
            animator.sensor_update_degree(bone_name, axis_name, axis.angle);
            animator.rotate_all();
            //console.log("Test")
        });
    });

    setTimeout(sendNewAngle, 15);
}

setTimeout(sendNewAngle, 1000);