import { initializeApp } from 'firebase/app';
import { getFirestore, doc,getDoc} from 'firebase/firestore';

// Your web app's Firebase configuration
const firebaseConfig = {
    apiKey: "AIzaSyDKuF2rvUTePEPGIad4v3t68fzIRMX897E",
    authDomain: "embed-2024.firebaseapp.com",
    projectId: "embed-2024",
    storageBucket: "embed-2024.appspot.com",
    messagingSenderId: "847089588926",
    appId: "1:847089588926:web:239fbf04ea632ed9e9dcef"
};

// Initialize Firebase
const app = initializeApp(firebaseConfig);
const db = getFirestore(app);

const docRef = doc(db, "data","test");
const docSnap = await getDoc(docRef);

if (docSnap.exists()) {
  console.log("Document data:", docSnap.data());
} else {
  console.log("No such document!");
}
