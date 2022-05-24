<script lang="ts">
	import funcs from "../lib/funcs";
	let num = 0;
	let speed = 0;
	let acc = 0.0;
	setInterval(() => {
		speed += acc;
		speed -= 0.0001 * speed; // friction
		num += speed;
	}, 5);
	let rotation = 0;
	let prevnum = num;
	setInterval(() => {
		rotation -= (prevnum - num * (1 - (Math.random()/500))) / 250;
		prevnum = num
	}, 5);
	funcs(
		(e: number) => {
			acc = e/20000;
		},
		() => {
			return rotation;
		},
		() => {
			return num;
		}
	);
</script>

<div class="wrapp">
<div class="ball" style="left: {Math.abs(num % 100)}vw" />
<div class="rot" style="transform: translate(-50%, -50%) rotate({rotation}turn)"></div>
</div>
<style>
	.ball {
		width: 10px;
		height: 10px;
		border-radius: 50%;
		background-color: red;
		position: fixed;
		top: 0px;
	}
	.wrapp {
		width: 100vw;
		height: 100vh;
		background-color: dimgray;
		margin: none;
		padding: none;
		position: fixed;
		top: 0px;
		left: 0px;
	}
	.rot {
		width: 10px;
		height: 100px;
		position: fixed;
		left: 50%;
		top: 50%;
		background-color: black;
	}
</style>
