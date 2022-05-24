let target = 50;
let preverr = 0;
const p = 10;
const d = 800;

let rotarget = 0.5;
let rotpreverr = 0;
const rotp = 50000;
const rotd = 50000;
export default function profile(setacc: (e: number) => void, getrot: () => number, getpos: () => number) {
    preverr = (target - getpos());
    // wheel pid
    setInterval(() => {
        const error = (target - getpos());
        setacc((error * p) + ((error - preverr) * d));
        preverr = error;
    }, 5)
    // set target
    setInterval(() => {
        rotarget = Math.sin(performance.now()/1000);
    }, 5)
    setInterval(() => {
        const error = (rotarget - getrot());
        setacc(error * rotp + (error - rotpreverr) * rotd);
        rotpreverr = error;
    }, 5)
}