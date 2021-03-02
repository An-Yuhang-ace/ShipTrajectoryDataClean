import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * The type Trajectory point.
 * 航迹类表示单个航迹点，包括轨迹的时空间信息和时空间增量。
 *
 * @author An Yuhang
 * @version 1.0
 * @date 2021 /1/20 15:33
 */
public class TrajectoryPoint {
    SimpleDateFormat df = new SimpleDateFormat("dd/MM/yyyy HH:mm:ss");

    public String mmsi;
    /** Date格式时间 */
    public Date date;
    /** UTC时间,单位ms */
    public long time;
    /** 时间增量,单位ms。 若 == 0 则代表为轨迹起始点 */
    public long deltatime = 0;
    public double lng, lat;
    public double cog, sog;
    /** 经纬度增量 */
    public double deltalng = 0.0, deltalat = 0.0;

    public TrajectoryPoint(String input){
        //根据读取到的轨迹点数据创建一个TrajectoryPoint对象
        input = input.replace("\"","");
        String[] line = input.split(",");
        try{
            this.date = df.parse(line[0]);
        } catch (Exception e) {
            e.printStackTrace();
        }
        this.time = date.getTime();
        this.mmsi = line[1];
        this.lng = Double.parseDouble(line[4]);
        this.lat = Double.parseDouble(line[5]);
        this.sog = Double.parseDouble(line[3]);
        this.cog = Double.parseDouble(line[6]);
    }




    public static void main(String[] arg) {
        //test
        String str1 = "27/7/2020 11:10:29";
        String str2 = "27/7/2020 11:10:39";
        SimpleDateFormat df = new SimpleDateFormat("dd/MM/yyyy HH:mm:ss");
        Date date1 = null;
        Date date2 = null;
        try {
            date1 = df.parse(str1);
            date2 = df.parse(str2);
        } catch (Exception e) {
            e.printStackTrace();
        }
        if (date2 != null && date1 != null){
            System.out.println((date2.getTime() - date1.getTime()));
        }
    }

}