import java.io.*;
import java.net.*;

public class Server {
    public static void main(String[] args) {
        int port = 12345; //initialze port number

        //creates a listening end on the specified port number
        try (ServerSocket serverSocket = new ServerSocket(port)) {
            System.out.println("Server is listening on port " + port);

            //if socket is accepted, the client is connected
            Socket socket = serverSocket.accept();
            System.out.println("Client connected");

            //server can handle both reading and writing in order to ensure bidrectionality
            BufferedReader reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            PrintWriter writer = new PrintWriter(socket.getOutputStream(), true);

            //constantly polls for client messages while writing that the client message has been acknowledged 
            String clientMessage;
            while ((clientMessage = reader.readLine()) != null) {
                System.out.println("Received GPS data: " + clientMessage);
                writer.println("Acknowledged: " + clientMessage);
            }
            //when the data stops polling close the socket
            //still need to add a types message that will close it manually
            socket.close();
            System.out.println("Client disconnected");
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
}
