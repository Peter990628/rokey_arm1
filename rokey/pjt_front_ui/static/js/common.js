// ============================================================
// 공통 JS - 모든 페이지에서 공유
// ============================================================

const socket = io();

// ---- 시계 ----
function tickClock() {
  const el = document.getElementById('clock');
  if (el) el.textContent = '🕐 ' + new Date().toLocaleTimeString('ko-KR', { hour12: false });
}
setInterval(tickClock, 1000);
tickClock();

// ---- 전역 재고부족 배너 (모든 페이지 공통, 소리 없이 시각적 알림만) ----
socket.on('stock_alert', (payload) => {
  const banner = document.getElementById('alertBanner');
  const text = document.getElementById('alertText');
  if (!banner || !text) return;
  text.textContent = `${payload.med_name || payload.med_id} 남은 수량 ${payload.count}개`;
  banner.classList.add('show');
  setTimeout(() => banner.classList.remove('show'), 4000);
});

// ---- 처방전 등록 시 재고 부족으로 거절된 경우 (프론트 폼 제출 응답과 별개로,
//      다른 화면에 이미 열려있는 사용자에게도 알리기 위한 브로드캐스트) ----
socket.on('prescription_rejected', (payload) => {
  const banner = document.getElementById('alertBanner');
  const text = document.getElementById('alertText');
  if (!banner || !text) return;
  text.textContent = `처방전 등록 실패 — ${payload.reason || '재고 부족'}`;
  banner.classList.add('show');
  setTimeout(() => banner.classList.remove('show'), 4000);
});
