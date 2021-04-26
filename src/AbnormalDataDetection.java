import java.io.*;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.List;

/**
 * 检测并清洗轨迹序列中的异常数据并进行轨迹划分
 * 包括功能：添加增量，轨迹划分，轨迹去噪，偏移处理，稀疏处理。
 *
 * 1，添加增量：
 * 通过读取csv格式的航迹数据，为轨迹点添加时空间增量（经纬度增量，时间增量）
 * 2，轨迹划分：
 * 设定时间间隔阈值，将间隔较大的点删除并添加分段点；将停泊点设为分段点。
 * 分段点的时空间增量值都为0.
 * 3，轨迹去噪：
 * 将时间重复轨迹点，异常时间轨迹点去除
 * 4，偏移处理：
 * 检测轨迹序列中的偏移点（离群点），通过分箱-3西格玛检测，另外设定增量阈值进行检测
 * 对异常点进行清洗：非连续异常点利用线性插补发进行修复，无法插补则删除。
 * 5，稀疏处理：
 * 设定长度阈值，将长度长于阈值的轨迹点输出至csv文件中。
 *
 * @author An Yuhang
 * @version 3.0
 * @date 2021/4/2 15:41
 */
public class AbnormalDataDetection {

    /**
     * The entry point of application.
     *
     * @param args the input arguments
     */
    public static void main(String[] args) {
        long beginTime = System.currentTimeMillis();
        List<TrajectoryPoint> trajectory = new ArrayList<>();
        try {
            //读取csv文件
            BufferedReader reader = new BufferedReader(new FileReader("ship_log_trajectory.csv"));
            //读第一行
            String line = reader.readLine();
            TrajectoryPoint t = new TrajectoryPoint(line);
            trajectory.add(t);
            TrajectoryPoint prior = t;
            //分段点阈值，时间间隔大于该值的点将被视为分段点，单位s。
            int breakThreshold = 20;
            while((line=reader.readLine())!=null ){
                //按行读取文件中的航迹信息，根据mmsi将其构建为轨迹序列
                if (line.equals("")) {
                    continue;
                }
                t = new TrajectoryPoint(line);
                if (t.mmsi.equals(prior.mmsi)) {
                    /*
                       在同一船舶轨迹点间，根据前一轨迹点，添加时间、经度、纬度增量。
                       if 时间间隔大于breakThreshold，则时间增量为0，代表分段点。
                       if 与前一点时间重复，跳过。
                       if 时间增量为负，证明前一点为误码点，将前一点从序列中删除，更新prior。
                    */
                    if (t.time - prior.time < breakThreshold * 1000 && t.time - prior.time > 0 ){
                        t.deltatime = t.time - prior.time;
                        t.deltalng = t.lng - prior.lng;
                        t.deltalat = t.lat - prior.lat;
                        if (t.deltalng == 0.0 && t.deltalat == 0.0){
                            //经纬度增量都为0，停泊点，时间增量设为0，作为分段点
                            t.deltatime = 0;
                        }
                    }
                    else if (t.time == prior.time){
                        continue;
                    }
                    else if (t.time < prior.time){
                        if (trajectory.size() > 1){
                            trajectory.remove(trajectory.size()-1);
                            prior = trajectory.get(trajectory.size()-1);
                            t.deltatime = t.time - prior.time;
                            t.deltalng = t.lng - prior.lng;
                            t.deltalat = t.lat - prior.lat;
                        }
                        else{
                            continue;
                        }
                    }
                    prior = t;
                    trajectory.add(t);
                } else {
                    //单个船舶的航迹读取和增量添加结束，开始进行异常数据处理。
                    outliersCorrect(trajectory);
                    //将处理后数据输出
                    trajectoryWrite(trajectory);
                    trajectory.clear();
                    trajectory.add(t);
                    prior = t;
                }
            }
            //读取结束后还要对最后一个序列进行处理和输出。
            outliersCorrect(trajectory);
            //abnormalTurningCorrect(trajectory);
            //sparseTrajectoryDelete(trajectory);
            trajectoryWrite(trajectory);
            trajectory.clear();
            long endTime = System.currentTimeMillis();
            System.out.println(String.format("处理用时%dms", (endTime-beginTime)));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    /**
     * 用分箱法检测轨迹序列中的偏移轨迹点（离群点），由于离群点会造成麻烦的影响，我们直接将其设为断点。
     *
     * @param trajectory the trajectory
     */
    public static void outliersCorrect(List<TrajectoryPoint> trajectory){
        //用宽度为k的箱子来分装轨迹点
        int binWidth = 40;
        int index = 0;
        while(index < trajectory.size()){
            TrajectoryPoint[] bin = new TrajectoryPoint[binWidth];
            //依次将轨迹点装箱，并进行异常检测和修复
            double latSum = 0.0, lngSum = 0.0;
            for (int j = 0; j < binWidth && index < trajectory.size(); j++, index++){
                bin[j] = trajectory.get(index);
                latSum += bin[j].lat;
                lngSum += bin[j].lng;
            }
            //计算经纬度的均值和标准差
            double latMean = latSum / binWidth;
            double lngMean = lngSum / binWidth;
            double latSd = 0.0, lngSd = 0.0;
            for(int j = 0; j < binWidth; j++){
                if (bin[j] == null) {
                    break;
                }
                latSd += (bin[j].lat - latMean) * (bin[j].lat - latMean);
                lngSd += (bin[j].lng - lngMean) * (bin[j].lng - lngMean);
            }
            latSd = latSd / binWidth;
            lngSd = lngSd / binWidth;
            latSd = Math.sqrt(latSd);
            lngSd = Math.sqrt(lngSd);
            //进行异常漂移点检测
            for(int j = 0; j < binWidth; j++){
                if (bin[j]!= null &&
                   (threeSigmaDetection(bin[j], latMean, lngMean, latSd, lngSd) || tooBigDelta(bin[j]))){
                    //根据三西格马法则 或 绝对增量过大 判定为异常漂移点（离群点），进行修复
                    int position = index - binWidth + j;
                    TrajectoryPoint next = null;
                    if (j != 0 && j != binWidth-1){
                        next = bin[j+1];
                    }
                    else if (position - 1 >= 0 && position + 1 < trajectory.size()){
                        next = trajectory.get(position + 1);
                    }
                    if (next != null){
                        next.deltatime = next.deltatime + bin[j].deltatime;
                        next.deltalat = next.deltalat + bin[j].deltalat;
                        next.deltalng = next.deltalng + bin[j].deltalng;
                        if (next.deltatime > 30000){
                            next.deltatime = 0;
                            next.deltalat = 0.0;
                            next.deltalng = 0.0;
                        }
                    }
                    bin[j].deltatime = 0;
                    bin[j].deltalat = 0.0;
                    bin[j].deltalng = 0.0;
                }
            }
        }
    }

    private static boolean threeSigmaDetection(TrajectoryPoint node, double latMean, double lngMean, double latSd, double lngSd){
        if (Math.abs(node.lat - latMean) > 3 * latSd){
            return true;
        }
        else if (Math.abs(node.lng - lngMean) > 3  * lngSd){
            return true;
        }
        else {
            return false;
        }
    }

    private  static  boolean tooBigDelta(TrajectoryPoint node){
        if (Math.abs(node.deltalat) > 0.002){
            return true;
        }
        else if (Math.abs(node.deltalng) > 0.002){
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * 根据两点的经纬度计算线性插入点的对地航向 cog
     *
     * @param latA 纬度1
     * @param lngA 经度1
     * @param latB 纬度2
     * @param lngB 经度2
     * @return cog
     */
    private static double getAngle(double latA, double lngA, double latB, double lngB) {

        double y = Math.sin(lngB-lngA) * Math.cos(latB);
        double x = Math.cos(latA)*Math.sin(latB) - Math.sin(latA)*Math.cos(latB)*Math.cos(lngB-lngA);
        double cog = Math.atan2(y, x);
        cog = Math.toDegrees(cog);
        if(cog < 0){
            cog = cog +360;
        }
        return cog;
    }

    /**
     * 检测并修复一个轨迹序列中的异常停泊点。
     * TODO 通过调整时间阈值解决，如果之后有必要精细处理再说
     *
     * @param trajectory the trajectory
     */
    public static void abnormalTurningCorrect(List<TrajectoryPoint> trajectory){

    }


    /**
     * 判断轨迹序列连续段的长度，长度长于阈值 threshold 的轨迹段写入csv文件中
     *
     * @param trajectory the trajectory
     */
    public static void trajectoryWrite(List<TrajectoryPoint> trajectory){
        try{
            File csv = new File("Trajectory.csv");
            int threshold = 60;
            BufferedWriter bw = new BufferedWriter(new FileWriter(csv, true));
            int count = 0;
            for (int i = 0; i < trajectory.size(); i++){
                TrajectoryPoint node = trajectory.get(i);
                count++;
                if (node.deltatime != 0){
                    if (count > threshold){
                        pointWrite(node, bw);
                    }
                    else if (count == threshold){
                        for (int j = 0; j < threshold; j++){
                            pointWrite(trajectory.get(i - threshold + 1 + j), bw);
                        }
                    }
                }
                else{
                    count = 1;
                }
            }
            bw.close();
        } catch (FileNotFoundException e) {
            // File对象的创建过程中的异常捕获
            e.printStackTrace();
        }catch (IOException e) {
            // BufferedWriter在关闭对象捕捉异常
            e.printStackTrace();
        }
    }

    /**
     * 将一个TrajectoryPoint对象的数据写入目标csv文件。
     *
     * @param a  the Trajectory point
     * @param bw 输出文件
     */
    public static void pointWrite(TrajectoryPoint a, BufferedWriter bw){
        SimpleDateFormat df = new SimpleDateFormat("dd/MM/yyyy HH:mm:ss");
        try{
            String dateString = df.format(a.date);
            bw.write(String.format("\"%s\",\"%s\",\"%f\",\"%f\",\"%f\",\"%f\",\"%d\",\"%f\",\"%f\"", dateString, a.mmsi, a.sog, a.lng, a.lat, a.cog, a.deltatime, a.deltalng, a.deltalat));
            bw.newLine();
        } catch (IOException e) {
            // BufferedWriter在关闭对象捕捉异常
            e.printStackTrace();
        }
    }

}