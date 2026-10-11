// Empty resource links stay inert; adding an href activates them automatically.
document.querySelectorAll('a[href=""]').forEach(link => {
  link.addEventListener('click', event => {
    if (!link.getAttribute('href')?.trim()) event.preventDefault();
  });
});

const dialog = document.querySelector('.image-dialog');
let priorFocus;
document.querySelectorAll('[data-zoom]').forEach(button => button.addEventListener('click', () => {
  const source = button.querySelector('img');
  priorFocus = button;
  const expanded = dialog.querySelector('img');
  expanded.src = source.src;
  expanded.alt = source.alt;
  document.getElementById('image-dialog-title').textContent = button.dataset.zoom;
  dialog.showModal();
  dialog.querySelector('.dialog-scroll').scrollTo({top:0,left:0});
  document.body.style.overflow = 'hidden';
}));
dialog.querySelector('.dialog-close').addEventListener('click', () => dialog.close());
dialog.addEventListener('click', event => { if (event.target === dialog) dialog.close(); });
dialog.addEventListener('close', () => {document.body.style.overflow = ''; if (priorFocus) priorFocus.focus();});
const copyButton = document.getElementById('copy-citation');
if (copyButton) copyButton.addEventListener('click', async () => {
  const text = document.getElementById('bibtex').textContent;
  const status = document.getElementById('copy-status');
  try {
    if (navigator.clipboard && window.isSecureContext) await navigator.clipboard.writeText(text);
    else {
      const field = document.createElement('textarea');
      field.value = text; field.style.position = 'fixed'; field.style.opacity = '0';
      document.body.appendChild(field);field.select();
      const copied = document.execCommand('copy');field.remove();
      if (!copied) throw new Error('Copy unavailable');
    }
    status.textContent = 'Citation copied.';
  } catch {status.textContent = 'Select and copy the citation below.';}
});
