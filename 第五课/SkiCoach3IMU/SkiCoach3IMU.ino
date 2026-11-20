// ============================================================================
// 文件名：SkiCoach3IMU.ino
// 它是谁？—— 第五课 ESP32 主控板上的总指挥程序（硬件收发与状态控制层）
//
// 【给中学生的阅读指南】
// 如果说 Posture.h 是专门做数学题的“大脑”，那么这个 SkiCoach3IMU.ino 文件就是机器人的
// “耳朵、嘴巴和总开关”：
//   1. 开机时（setup）：让 ESP32 变成一个名叫 "SKI-COACH" 的 Wi-Fi 热点，打开 1399 号窗口等数据。
//   2. 运行时（loop）：
//      • 检查有没有哪枚传感器超过 0.1 秒没说话（掉线了就报 OFFLINE）；
//      • 听电脑串口发来的指挥口令（CAL = 站直校准，START = 开始监测，STOP = 停止监测）；
//      • 从 Wi-Fi 空气里抓到一包 UDP 数据后，交给 Posture.h 的 decodeFrame() 拆包；
//      • 认出这包数据是大腿(T)、小腿(S)还是腰部(W)发来的，分别送去算膝角 K、算腰部晃动 W，
//        一旦违规超时就打印报警事件 E！
//
// 【串口打印的 6 种字母代号（波特率 921600）】
//   • R        (Raw 原始角)  ：R,时间ms,部位T/S/W,设备编号,X角,Y角,Z角  （只要传感器开机连上就会一直打）
//   • K        (Knee 膝角)   ：K,时间ms,大腿角,小腿角,膝盖弯曲角K,两帧时差ms （做完 CAL 校准后开始打）
//   • W        (Waist 腰晃)  ：W,时间ms,前后倾,左右倾,1秒晃动幅度W       （发了 START 且攒满 1 秒后开始打）
//   • E        (Event 报警)  ：E,时间ms,报警名称,触发时的度数            （发了 START 且违规满时间才打）
//   • STATUS   (系统状态)    ：上线 ONLINE、掉线 OFFLINE、校准成功 CAL_OK 等提示
//   • DISCOVER (发现新设备)  ：收到还没登记进 DEVICE_ID 表的陌生传感器编号时打印，帮你找新传感器 ID
// ============================================================================

#include <WiFi.h>      // ESP32 自带的 Wi-Fi 功能库（用来开热点）
#include <WiFiUdp.h>   // ESP32 自带的 UDP 无线收发库（用来收传感器发来的数据包）
#include "Posture.h"   // 引入我们同文件夹下的“纯数学算法大脑”头文件

// ============================================================================
// 第一部分：Wi-Fi 热点设置（三枚传感器开机后都会自动找这个热点连进来）
// ============================================================================
const char *AP_NAME = "SKI-COACH";        // ESP32 放出的 Wi-Fi 热点名字
const char *AP_PASSWORD = "ski12345";     // Wi-Fi 热点密码（至少 8 位）
const uint16_t UDP_PORT = 1399;           // 约定的收件箱端口号：传感器都往 1399 端口投递数据
const IPAddress AP_IP(192, 168, 10, 1);   // ESP32 自己的局域网 IP 地址（192.168.10.1）
const IPAddress MASK(255, 255, 255, 0);   // 子网掩码（规定 192.168.10.x 网段是一家人）

// ============================================================================
// 第二部分：三枚传感器的“身份证号登记表”与穿戴部位
// ============================================================================
// 数组下标 0、1、2 分别对应三个身体部位：
//   • 下标 0 -> 'T' (Thigh 大腿)：绑在一条腿的大腿中段外侧
//   • 下标 1 -> 'S' (Shank 小腿)：绑在【同一条腿】的小腿外侧（必须和大腿绑在同一侧才能算这个膝盖的角！）
//   • 下标 2 -> 'W' (Waist 腰部)：绑在后腰正中间贴近骨盆的位置
//
// 【⚠️ 上课必做第一步】：
// 前两枚传感器（大腿 14327、小腿 14444）已经填好了，但第 3 枚腰部传感器现在填的是空字符串 ""！
// 请先把本程序烧进 ESP32，打开串口监视器并打开第 3 枚传感器电源，串口会打印：
//   DISCOVER,时间,WT550001xxxx
// 把看到的 12 位编号填进下面第三个引号 "" 里，再重新点一次上传，腰部传感器就正式转正啦！
const char *DEVICE_ID[3] = {"WT5500014327", "WT5500014444", ""};
const char ROLE[3] = {'T', 'S', 'W'};

// ============================================================================
// 第三部分：传感器的轴向（X/Y/Z）与正负方向设置（上机绑好后需核对）
// ============================================================================
// 每枚传感器都会同时输出 0(X轴)、1(Y轴)、2(Z轴) 三个角度。
// 但传感器绑在腿上或腰上时，可能是竖着绑、横着绑、正着绑或倒着绑的。
// 所以我们要通过实验告诉程序：“屈膝时应该看哪一根轴？往哪个方向转算正数？”
//
// 【核对方法】：
// 1. 只转动大腿或小腿，盯住串口 R 行的最后三个数（X、Y、Z），哪一列变化最大，就把 AXIS 设为 0、1 或 2。
// 2. 乘 +1 保持原来的正负号，乘 -1 把正负号反过来。
//    目标只有一个：【大腿和小腿绑在同一块板上一起转时，它们的变化方向必须相同；人屈膝蹲下时，算出的膝角 K 必须变大！】
// 3. 腰部传感器需要看两个方向：前后弯腰俯仰（PITCH）看一根轴，左右歪腰侧倾（ROLL）看另一根轴。
constexpr size_t THIGH_AXIS = 0;        // 大腿屈伸主轴（默认 0 即 X 轴）
constexpr size_t SHANK_AXIS = 0;        // 小腿屈伸主轴（默认 0 即 X 轴）
constexpr float THIGH_SIGN = 1;         // 大腿角度方向符号（+1 或 -1）
constexpr float SHANK_SIGN = 1;         // 小腿角度方向符号（+1 或 -1）
constexpr size_t WAIST_PITCH_AXIS = 1;  // 腰部前后倾对应的轴（默认 1 即 Y 轴）
constexpr size_t WAIST_ROLL_AXIS = 0;   // 腰部左右倾对应的轴（默认 0 即 X 轴）
constexpr float WAIST_PITCH_SIGN = 1;   // 腰部前后倾方向符号（+1 或 -1）
constexpr float WAIST_ROLL_SIGN = 1;    // 腰部左右倾方向符号（+1 或 -1）

// ============================================================================
// 第四部分：全局变量与 Posture.h 算法工人（程序运行时的记忆区）
// ============================================================================
WiFiUDP udp;                            // 负责收发 UDP 数据包的邮递员
ski::Pairer legs;                       // 第 2 关工人：专门负责把大腿 T 和小腿 S 按 12ms 时差配成一对
ski::WaistWindow waist;                 // 第 4 关工人：专门负责攒满腰部最近 1 秒数据并算晃动幅度 W
ski::Hold kneeLow, kneeHigh, waistHigh; // 第 5 关工人：三只互相独立的裁判秒表（分别盯：腿太直、蹲太深、腰乱晃）

// 在线状态与掉线监控变量：
bool seen[3] = {};                      // seen[0/1/2] 记录 T/S/W 三枚传感器现在是不是在线（true=在线，false=离线）
uint32_t lastRx[3] = {};                // lastRx[0/1/2] 记录最后一次收到 T/S/W 数据是哪一毫秒（用来查有没有超过 100ms 没说话）
uint32_t nextUnknownReport = 0;         // 防止未登记传感器刷屏的计时器：每隔 500 毫秒最多打印一次 DISCOVER
bool udpReady = false;                  // 记录开机时 Wi-Fi 热点和 UDP 端口有没有成功打开

// 校准（CAL）与滑行监测（START / STOP）的状态开关：
bool running = false;                   // 当前是不是处于 START 开启的监测状态？（只有它为 true 时才打印 W 行和报警 E 行）
bool calibrated = false;                // 腿部是不是已经做完 CAL 站直校准了？（没校准前不知道零偏，不能算膝角 K，也不准 START）
bool calibrating = false;               // 当前是不是正在做 CAL 校准（正在凑 40 对站直数据的过程中）？
uint32_t calBegin = 0;                  // 按下 CAL 开始校准的那一毫秒（如果超过 8 秒还没站稳凑够 40 对，就报超时退出）

// 站直校准算零偏用的 4 个数学变量：
float zeroDiff = 0;                     // 校准最终算出来的“站直时 小腿角 - 大腿角”天生安装差值（也就是公式里的 standingDiff）
float calAnchor = 0;                    // 校准开始后收到的“第 1 对”角差基准（用来比对后面 39 对数据有没有乱动）
float calSum = 0;                       // 校准期间 40 对数据相对于第 1 对基准的变化量总和（用来求平均）
unsigned calCount = 0;                  // 校准期间已经攒了多少对合格的站直数据了？（攒够 40 对就校准成功！）

// 腿部配对流新鲜度监控：
uint32_t lastPair = 0;                  // 上一次大腿和小腿成功配成一对是哪一毫秒？
bool havePair = false;                  // 目前有没有正在持续配对的腿部数据流？

// 接收电脑串口命令（比如你输入 CAL 然后按回车）的文字篮子：
char command[32] = {};                  // 最多装 31 个字母的字符数组
size_t commandLength = 0;               // 当前篮子里已经装了几个字母
bool commandOverflow = false;           // 如果你一口气敲了超过 31 个字母没按回车，就标记为“装不下溢出了”

// ============================================================================
// 第五部分：状态清零、掉线巡检与站直校准函数
// ============================================================================

// 函数：resetKnee —— 清空腿部配对室和两只膝角秒表（但保留已经校准好的站直零偏 zeroDiff）。
// 什么时候调用？当大腿或小腿掉线、或者刚发 CAL / START / STOP 时调用，确保不会拿着断线前的旧秒表继续算。
void resetKnee() {
    legs.reset();       // 清空大腿和小腿的配对等候椅
    kneeLow.reset();    // 清零“腿太直”裁判秒表
    kneeHigh.reset();   // 清零“蹲太深”裁判秒表
    havePair = false;   // 标记当前没有配对流
    calCount = 0;       // 如果正在校准中掉线了，已攒的校准次数也归零重来
}

// 函数：resetWaist —— 清空腰部的 1 秒滑动窗口和“腰乱晃”裁判秒表。
// 什么时候调用？当腰部掉线、或者重新发 START / STOP 时调用，重连后必须重新攒满 1 秒数据。
void resetWaist() {
    waist.clear();      // 清空腰部 1 秒窗口里的所有历史样本
    waistHigh.reset();  // 清零“腰乱晃”裁判秒表
}

// 函数：expireStreams —— 巡逻检查有没有传感器掉线、配对中断或校准超时。
// 这个函数在主循环 loop() 里每一圈都会被调用，哪怕当前没有任何无线数据包飞过来也会按时检查！
void expireStreams(uint32_t now) {
    // 1. 挨个检查 0(大腿T)、1(小腿S)、2(腰部W) 三枚传感器：
    for (size_t i = 0; i < 3; ++i) {
        // 如果它本来就不在线（!seen[i]），或者距离上次收到它说话还没超过 100 毫秒（GapMs），那就一切正常，看下一个。
        if (!seen[i] || now - lastRx[i] <= ski::GapMs) continue;

        // 走到这里，说明这枚传感器刚才还在线，但现在已经超过 100 毫秒没发来任何数据了——判定为掉线（OFFLINE）！
        seen[i] = false;              // 把它的在线牌子摘掉
        if (i == 2) resetWaist();     // 如果是 2 号（腰部 W）掉线，清空腰部窗口和秒表
        else resetKnee();             // 如果是 0 号或 1 号（大腿或小腿）掉线，清空腿部配对室和膝角秒表
        // 在串口打印掉线通知，比如：STATUS,4500,W,OFFLINE
        Serial.printf("STATUS,%lu,%c,OFFLINE\n", (unsigned long)now, ROLE[i]);
    }

    // 2. 一种隐藏的特殊情况：假如大腿和小腿都在发数据（都没掉线），但它们俩到站的时间差老是超过 12 毫秒配不上对！
    //    如果超过 100 毫秒没有产出新的配对结果，也要清空膝角秒表，防止秒表一直卡在旧状态。
    if (havePair && now - lastPair > ski::GapMs) resetKnee();

    // 3. 校准超时检查：如果你发送了 CAL 命令开始校准，但过了 8000 毫秒（8 秒钟）还没凑够 40 对稳定的腿部数据
    //    （比如腿一直在晃，或者有一枚腿部传感器根本没开机），就自动取消校准，打印 STATUS,CAL_TIMEOUT。
    if (calibrating && now - calBegin > 8000) {
        calibrating = false;
        Serial.println("STATUS,CAL_TIMEOUT");
    }
}

// 函数：event —— 当某只裁判秒表满时间吹哨时，往串口打印一条 E 报警事件。
// 例如：E,1800,KNEE_LOW,10.00 （表示在 1800 毫秒时触发了腿太直报警，当时膝角只有 10.00 度）。
void event(uint32_t now, const char *kind, float value) {
    Serial.printf("E,%lu,%s,%.2f\n", (unsigned long)now, kind, value);
}

// 函数：updateCalibration —— 站直校准（CAL）时，一步步收集 40 对稳定的“小腿角 - 大腿角”求平均零偏。
// 参数 diff 是刚刚配对成功的一对“小腿角 - 大腿角”。
void updateCalibration(uint32_t now, float diff) {
    // 如果是校准开始后的第 1 对数据（calCount == 0），就把它记作本轮的“基准锚点 calAnchor”。
    if (!calCount) calAnchor = diff;

    // 算一算当前这一对角差（diff）跟第 1 对基准锚点（calAnchor）偏了多少度？
    // 同样套上 wrap180，防止剛好在 ±180° 分界线两边差 2° 被误当成差了 358°。
    const float fromAnchor = ski::wrap180(diff - calAnchor);

    // 【防乱动检查】：如果在站直校准的过程中，你的腿晃动超过了 ±4.0 度（fabs 是求绝对值），
    // 说明你没站稳！刚才攒的数据全都不作数（calCount = 0, calSum = 0），打印提示让你站稳自动重攒！
    if (std::fabs(fromAnchor) > 4.0f) {
        calCount = 0;
        calSum = 0;
        Serial.println("STATUS,CAL_MOVED_RETRY");
        return;
    }

    // 站得很稳（偏差在 4° 以内）！把这次相对于基准的微小偏差加到总和 calSum 里，已攒对数 calCount 加 1。
    if (!calCount) calSum = 0;
    calSum += fromAnchor;

    // 如果还没攒够 40 对（正常每秒 50 对，40 对大约需要 0.8 秒），就先返回，继续等下一对数据。
    if (++calCount < 40) return;

    // 攒够 40 对啦！
    // 用“第 1 对基准锚点 calAnchor + 40 对微小偏差的平均值 (calSum / 40)”，再折返进 [-180°, 180°)，
    // 就得到了超级准的站直天生角差 zeroDiff！
    zeroDiff = ski::wrap180(calAnchor + calSum / calCount);
    calibrated = true;   // 盖章通过：已经校准好啦！
    calibrating = false; // 结束“正在校准”状态
    // 打印校准成功及算出的零偏度数，比如：STATUS,800,CAL_OK,6.00
    Serial.printf("STATUS,%lu,CAL_OK,%.2f\n", (unsigned long)now, zeroDiff);
}

// ============================================================================
// 第六部分：核心枢纽 onFrame() —— 每收到一帧合法传感器数据后怎么分流处理？
// ============================================================================
void onFrame(const ski::Frame &frame, uint32_t now) {
    // -------------------- 第 1 步：查户口（认出它是大腿、小腿还是腰部） --------------------
    // 拿拆包得到的 frame.id 跟我们的登记表 DEVICE_ID[0..2] 挨个比对（strcmp 返回 0 表示两个字符串一模一样）。
    size_t sensor = 0;
    for (; sensor < 3; ++sensor) {
        if (std::strcmp(frame.id, DEVICE_ID[sensor]) == 0) break;
    }

    // 如果比对完 0、1、2 都没对上（循环结束时 sensor 等于 3），说明这是一枚还没登记进 DEVICE_ID 表的陌生传感器！
    // 每隔 500 毫秒在串口打印一次 DISCOVER,时间,传感器编号，提醒你把编号抄进代码里，然后直接返回不往下走。
    if (sensor == 3) {
        if ((int32_t)(now - nextUnknownReport) >= 0) {
            Serial.printf("DISCOVER,%lu,%s\n", (unsigned long)now, frame.id);
            nextUnknownReport = now + 500;
        }
        return;
    }

    // -------------------- 第 2 步：更新在线状态并打印 R（原始角度）行 --------------------
    // 先按当前时间 now 检查一下有没有人已经超过 100ms 掉线（如果刚才掉线过，必须先清零旧秒表，再重新报上线 ONLINE）。
    expireStreams(now);
    if (!seen[sensor]) {
        // 如果它之前是不在线状态，现在突然收到它的数据了，打印一条上线通知：STATUS,时间,T/S/W,ONLINE
        Serial.printf("STATUS,%lu,%c,ONLINE\n", (unsigned long)now, ROLE[sensor]);
    }
    seen[sensor] = true; // 标记为在线
    lastRx[sensor] = now; // 更新它最后一次说话的时间戳

    // 不管有没有校准、有没有发 START，只要收到合法帧，永远打印 R 行（包含部位 T/S/W、设备号和 X/Y/Z 原始三轴角度）。
    Serial.printf("R,%lu,%c,%s,%.2f,%.2f,%.2f\n", (unsigned long)now,
                  ROLE[sensor], frame.id, frame.angle[0], frame.angle[1], frame.angle[2]);

    // -------------------- 第 3 步（分支 A）：如果这帧数据来自 2 号【腰部传感器 W】 --------------------
    if (sensor == 2) {
        // 腰部的波动计算和报警，只有在你发送了 START 命令开启监测后（running == true）才干活。
        if (!running) return;

        float wave = 0;
        // 按照我们在代码顶部设好的轴号和正负号，取出腰部的前后俯仰角(pitch)和左右侧倾角(roll)。
        const float pitch = WAIST_PITCH_SIGN * frame.angle[WAIST_PITCH_AXIS];
        const float roll = WAIST_ROLL_SIGN * frame.angle[WAIST_ROLL_AXIS];

        // 把这帧腰部数据送进【第 4 关：WaistWindow 1秒滑动窗口】：
        // 如果窗口还没攒满 1 秒或者有效帧数不够 35 个（push 返回 false），就把腰部秒表清零并先返回。
        if (!waist.push(now, pitch, roll, wave)) {
            waistHigh.reset();
            return;
        }

        // 窗口已满 1 秒，算出了波动幅度 wave！打印 W 行：W,时间ms,前后倾,左右倾,1秒波动W
        Serial.printf("W,%lu,%.2f,%.2f,%.2f\n", (unsigned long)now, pitch, roll, wave);

        // 送进【第 5 关：Hold 裁判秒表】：检查波动 wave 是不是大于 10°（WaistHigh），且连续保持满 500 毫秒？
        // 如果满 500 毫秒，吹哨打印一条骨盆晃动报警：E,时间ms,PELVIS_SWAY,波动度数
        if (waistHigh.update(now, wave > ski::WaistHigh, 500)) {
            event(now, "PELVIS_SWAY", wave);
        }
        return; // 腰部数据处理完毕！
    }

    // -------------------- 第 3 步（分支 B）：如果这帧数据来自 0 号【大腿 T】或 1 号【小腿 S】 --------------------
    // 1. 按照代码顶部设好的轴号和正负号，取出大腿或小腿的屈伸角度 segment。
    const float segment = sensor == 0 ? THIGH_SIGN * frame.angle[THIGH_AXIS]
                                      : SHANK_SIGN * frame.angle[SHANK_AXIS];
    uint32_t delta = 0;

    // 2. 送进【第 2 关：Pairer 12毫秒配对室】：
    //    如果另一条腿段还没到、或者两帧到达时差超过了 12ms（push 返回 false），那就没法算膝角，直接返回等下一帧。
    if (!legs.push(sensor, now, segment, delta)) return;

    // 3. 走到这里，说明大腿 T 和小腿 S 已经在 12 毫秒内成功牵手配成一对了！
    //    此时 legs.angle[0] 里是大腿角，legs.angle[1] 里是小腿角，delta 是它们俩相差的毫秒数。
    lastPair = now;
    havePair = true;

    // 4. 如果现在正处于 CAL 站直校准过程中（calibrating == true），就把这一对“小腿角 - 大腿角”送去攒够 40 对求零偏。
    if (calibrating) {
        updateCalibration(now, ski::wrap180(legs.angle[1] - legs.angle[0]));
    }
    // 如果还没完成过 CAL 校准（!calibrated），不知道站直零偏 zeroDiff 是多少，就不能往下算膝角 K。
    if (!calibrated) return;

    // 5. 已经校准过了！调用【第 3 关：kneeFlexion】算出扣除站直零偏后的真实膝盖弯曲角 angle（即讲义里的 K），
    //    并打印 K 行：K,时间ms,大腿角,小腿角,膝屈曲角K,两帧接收时差delta
    //    （贴心设计：只要做完了 CAL 校准，哪怕你还没发 START，串口也会一直打印 K 行，方便你在桌面上拿量角器做实验！）
    const float angle = ski::kneeFlexion(legs.angle[0], legs.angle[1], zeroDiff);
    Serial.printf("K,%lu,%.2f,%.2f,%.2f,%lu\n", (unsigned long)now, legs.angle[0],
                  legs.angle[1], angle, (unsigned long)delta);

    // 6. 如果还没发 START 命令开启滑行监测（!running），那么只打印 K 行看角度，不启动后面的裁判秒表报警。
    if (!running) return;

    // 7. 已开启 START 监测！把算出的膝角 angle 同时送给两只【第 5 关：Hold 裁判秒表】：
    //    • 规则 1（腿站太直）：如果膝角 < 20°（KneeLow）连续保持满 800 毫秒，报一次 KNEE_LOW！
    //    • 规则 2（膝蹲太深）：如果膝角 > 45°（KneeHigh）连续保持满 800 毫秒，报一次 KNEE_HIGH！
    if (kneeLow.update(now, angle < ski::KneeLow, 800)) event(now, "KNEE_LOW", angle);
    if (kneeHigh.update(now, angle > ski::KneeHigh, 800)) event(now, "KNEE_HIGH", angle);
}

// ============================================================================
// 第七部分：听取电脑串口发来的指挥命令（CAL / START / STOP）
// ============================================================================
// 在 Arduino 串口监视器上方输入框输入大写命令并按回车（右下角需选“换行符 Newline”）：
//   • CAL   ：开始站直校准（人要站直保持不动约 1 秒，等出现 STATUS,时间,CAL_OK,零偏）
//   • START ：开始本轮滑行监测（开始输出腰部 W 行和三种 E 报警事件）
//   • STOP  ：结束本轮滑行监测（关闭 E 报警和 W 行，清空所有秒表，但保留 R 行和 K 行）
void readCommand() {
    // 只要串口收件箱里有电脑发来的字符，就一个字母一个字母地读出来：
    while (Serial.available()) {
        const char c = (char)Serial.read();
        if (c == '\r') continue; // 忽略 Windows 回车符里的 '\r'

        // 如果还没读到换行符 '\n'，说明这一行命令还没敲完，先把字母放进 command 篮子里存着：
        if (c != '\n') {
            if (commandLength < sizeof(command) - 1) command[commandLength++] = c;
            else commandOverflow = true; // 敲的字母超过 31 个了，记下“溢出”
            continue;
        }

        // 读到了换行符 '\n'！说明你按下了回车键，一整条命令收齐了！
        command[commandLength] = '\0'; // 在末尾补上字符串结束符 '\0'
        commandLength = 0;             // 把篮子计数器归零，准备下次装下一条命令

        // 如果刚才敲得太长溢出了，整条命令直接扔掉作废，绝不把截断的半截命令当成真命令执行。
        if (commandOverflow) {
            commandOverflow = false;
            Serial.println("STATUS,COMMAND_TOO_LONG");
            continue;
        }

        // 比对篮子里的命令到底是不是 "CAL"、"START" 或 "STOP"：
        if (strcmp(command, "CAL") == 0) {
            running = false;       // 校准时必须先关掉监测状态
            calibrated = false;    // 旧的校准结果作废
            calibrating = true;    // 开启“正在校准”开关
            calBegin = millis();   // 记下校准开始的毫秒时刻（用来算 8 秒超时）
            resetKnee();           // 清空腿部配对室和秒表
            resetWaist();          // 清空腰部窗口和秒表
            Serial.println("STATUS,CAL_KEEP_STILL"); // 提醒你：请站直保持不动！
        } else if (strcmp(command, "START") == 0) {
            // 【安全检查门卫】：如果你还没做完 CAL 校准（!calibrated），或者大腿(!seen[0])、小腿(!seen[1])没在线，
            // 门卫会直接拒绝开启监测，并提示你先检查双腿传感器和校准！
            // （至于腰部传感器 seen[2]，允许稍后上线，上线后它自己会独立攒满 1 秒窗口。）
            if (!calibrated || !seen[0] || !seen[1]) {
                Serial.println("STATUS,START_REJECTED_CHECK_LEGS_AND_CAL");
                continue;
            }
            if (!running) {
                resetKnee();       // 每次新开一轮监测前，先把旧秒表清零
                resetWaist();      // 把旧腰部窗口清零
                running = true;    // 正式拉下总闸：开始滑行监测！
                Serial.printf("STATUS,%lu,SLIDE_START\n", (unsigned long)millis());
            }
        } else if (strcmp(command, "STOP") == 0) {
            running = false;       // 关掉监测总闸
            resetKnee();           // 清空腿部秒表
            resetWaist();          // 清空腰部窗口与秒表
            Serial.printf("STATUS,%lu,SLIDE_STOP\n", (unsigned long)millis());
        }
    }
}

// ============================================================================
// 第八部分：Arduino 必有的两大主函数 setup() 和 loop()
// ============================================================================

// setup()：在 ESP32 插上电或按复位键时，【只运行一次】的开机准备函数。
void setup() {
    // 1. 打开和电脑通信的串口，速度设为 921600（比普通 115200 快 8 倍，保证三路传感器每秒 150 行数据丝毫不卡顿）。
    Serial.begin(921600);

    // 2. 把 ESP32 的无线网卡设为 AP（Access Point，无线热点）模式。
    WiFi.mode(WIFI_AP);

    // 3. 设置热点的 IP 地址为 192.168.10.1，放出名叫 "SKI-COACH" 的 Wi-Fi 信号，并打开 1399 号 UDP 收件箱端口。
    const bool configured = WiFi.softAPConfig(AP_IP, AP_IP, MASK);
    udpReady = configured && WiFi.softAP(AP_NAME, AP_PASSWORD) && udp.begin(UDP_PORT);

    // 4. 如果三步都成功，串口打印 STATUS,UDP_READY；如果有一步失败，打印 STATUS,UDP_ERROR。
    Serial.printf("STATUS,UDP_%s\n", udpReady ? "READY" : "ERROR");
}

// loop()：在 setup() 跑完之后，ESP32 会以每秒几万次的速度【无限循环】跑这个函数。
void loop() {
    // 第 1 件事：看一眼现在的时间 millis()，巡检有没有哪枚传感器超过 100ms 没发数据（掉线了就报 OFFLINE）。
    expireStreams(millis());

    // 第 2 件事：看一眼电脑串口有没有发来 CAL、START、STOP 命令。
    readCommand();

    // 如果开机时 UDP 热点没启动成功，直接返回。
    if (!udpReady) return;

    // 第 3 件事：问一下 UDP 邮递员：“1399 号收件箱里收到新的无线数据包了吗？”
    // 如果 packetSize <= 0，说明这一瞬间没有新包，直接结束这一圈循环。
    const int packetSize = udp.parsePacket();
    if (packetSize <= 0) return;

    // 收到新包裹了！立刻看一眼 ESP32 手表，记下包裹到达的毫秒时刻 arrival。
    // （如果一个大 UDP 包里粘连了多帧数据，它们都共用这同一个到达时间 arrival。）
    const uint32_t arrival = millis();

    // 准备一个 1500 字节的大篮子（Wi-Fi 单个数据包通常最大就是 1500 字节），把包裹里的字节全都倒进 bytes 数组里。
    uint8_t bytes[1500];
    const int received = udp.read(bytes, sizeof(bytes));

    // 万一发来的超级大包裹超过了 1500 字节，把收件箱里剩下的尾巴读空扔掉，免得堵住下一包。
    while (udp.available() > 0) {
        if (udp.read() < 0) break;
    }

    // 如果包裹没读完整（被截断了），或者总长度连 1 帧（54 字节）都不够，直接扔掉不处理。
    if (packetSize != received || received < 54) return;

    // 第 4 件事：从包裹里一帧一帧地挑出 54 字节的传感器数据，送进【第 1 关：decodeFrame】安检拆包！
    //   • 如果从当前位置 offset 开始的 54 个字节成功拆出了一帧数据（decodeFrame 返回 true），
    //     就立刻交给 onFrame(frame, arrival) 去分流算角度，然后手指往后跳 54 个字节（offset += 54）看还有没有下一帧；
    //   • 如果当前位置不是合法的 'W' 'T' 帧头（decodeFrame 返回 false），
    //     手指就只往后挪 1 个字节（++offset），继续耐心地往后找真正的帧头。
    ski::Frame frame;
    for (size_t offset = 0; offset + 54 <= (size_t)received;) {
        if (ski::decodeFrame(bytes + offset, received - offset, frame)) {
            onFrame(frame, arrival);
            offset += 54;
        } else {
            ++offset;
        }
    }
}
