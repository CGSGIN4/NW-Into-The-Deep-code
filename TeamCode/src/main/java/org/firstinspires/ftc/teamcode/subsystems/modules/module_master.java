package org.firstinspires.ftc.teamcode.subsystems.modules;

import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.DUMMY_ACTION_CLOSE;
import static org.firstinspires.ftc.teamcode.subsystems.modules.module_master.action.DUMMY_ACTION_OPEN;

import org.firstinspires.ftc.teamcode.data.dataStorage;
import org.firstinspires.ftc.teamcode.subsystems.modules.test.dummyServo;

/* contains sample action could be done in auto. Dalshe ebites sami */
public class module_master {
    /* add actions here following byte flags conception */
    public enum action{;
        public static final int DUMMY_ACTION_CLOSE = 0b0000001;
        public static final int DUMMY_ACTION_OPEN = 0b0000010;
    }

    static dummyServo servak;

    public static void init(){
         servak = new dummyServo(dataStorage.OpMode.hardwareMap);
    }
    public static boolean doAction(int action){
        /* continue ifing this as shown */
        if ((action & DUMMY_ACTION_CLOSE) == DUMMY_ACTION_CLOSE){
            action -= DUMMY_ACTION_CLOSE;
            /* call function that opens claw */
            servak.launch();
        }

        if ((action & DUMMY_ACTION_OPEN) == DUMMY_ACTION_OPEN){
            action -= DUMMY_ACTION_OPEN;
            /* call function that opens claw */
            servak.prepare();
        }

        /* check if all actions are done */
        return action == 0;
    }

}
