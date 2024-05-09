import { initializeApp } from "firebase/app";
import { getFirestore} from 'firebase/firestore';

// Your web app's Firebase configuration
const firebaseConfig = {
  apiKey: "AIzaSyDKuF2rvUTePEPGIad4v3t68fzIRMX897E",
  authDomain: "embed-2024.firebaseapp.com",
  databaseURL: "https://embed-2024-default-rtdb.asia-southeast1.firebasedatabase.app",
  projectId: "embed-2024",
  storageBucket: "embed-2024.appspot.com",
  messagingSenderId: "847089588926",
  appId: "1:847089588926:web:239fbf04ea632ed9e9dcef"
};

// Initialize Firebase
export const fb = initializeApp(firebaseConfig);
export const db = getFirestore(fb);



