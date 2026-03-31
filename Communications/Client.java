import java.io.*;
import java.net.*;
import java.util.Random;

public class Client {
    public static void main(String[] args) {
        String hostname = "localhost"; //ip address of server
        int port = 12345; //port number

        //create the socket input ip address and port 
        try (Socket socket = new Socket(hostname, port)) {
            System.out.println("Connected to the server"); //print statement to validate connection 
            PrintWriter writer = new PrintWriter(socket.getOutputStream(), true);
            BufferedReader reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));

            //creating random mock gps data
            Random random = new Random();
            while (true) {
                double latitude = 37.7749 + random.nextDouble() * 0.01; 
                double longitude = -122.4194 + random.nextDouble() * 0.01;
                String gpsData = "{ \"latitude\": " + latitude + ", \"longitude\": " + longitude + " }";

                //prints out the sent gps data while also logging all sent coordinates
                writer.println(gpsData);
                System.out.println("Sent GPS data: " + gpsData);

                //prints out the servers response as well 
                String serverResponse = reader.readLine();
                System.out.println("Server response: " + serverResponse);
                Thread.sleep(1000);
            }

        } catch (UnknownHostException ex) {
            System.out.println("Server not found: " + ex.getMessage());
        } catch (IOException | InterruptedException ex) {
            System.out.println("Error: " + ex.getMessage());
        }
    }
}
